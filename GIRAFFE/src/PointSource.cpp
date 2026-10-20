#define NOMINMAX
#include "PointSource.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

#ifdef GIRAFFE_WITH_LASZIP
#include <laszip/laszip_api.h>   // je nach Paket auch <laszip_api.h> bzw. <laszip_dll.h>
#endif

namespace {

	std::string lower_ext(const std::string& path) {
		std::string e = std::filesystem::path(path).extension().string();
		std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		return e;
	}

	// ---------------------------------------------------------------- PW (wie bisher)
	// 1 Byte Header, danach pro Punkt: 3 x double (x,y,z) + 3 x uchar (r,g,b)
#pragma pack(push, 1)
	struct PwRec { double x, y, z; unsigned char c[3]; };
#pragma pack(pop)

	class PwSource : public PointSource {
	public:
		bool open(const std::string& path, std::string& error) override {
			_in.open(path, std::ios::binary);
			if (!_in) { error = "cannot open file: " + path; return false; }
			_in.seekg(0, std::ios::end);
			_size = static_cast<double>(_in.tellg());
			_in.seekg(1, std::ios::beg);                 // erstes Byte überspringen (wie bisher)
			_pos = 1;
			return true;
		}
		size_t read(std::vector<PointRecord>& out, size_t max_points) override {
			out.clear();
			_buf.resize(max_points * sizeof(PwRec));
			_in.read(_buf.data(), static_cast<std::streamsize>(_buf.size()));
			const size_t n = static_cast<size_t>(_in.gcount()) / sizeof(PwRec);   // unvollständiger Rest wird verworfen
			out.resize(n);
			for (size_t i = 0; i < n; ++i) {
				PwRec r;
				std::memcpy(&r, _buf.data() + i * sizeof(PwRec), sizeof(PwRec));
				out[i] = { r.x, r.y, r.z, r.c[0], r.c[1], r.c[2] };
			}
			_pos += static_cast<double>(n * sizeof(PwRec));
			return n;
		}
		double progress() const override { return _size > 0 ? std::min(1.0, _pos / _size) : 0.0; }
	private:
		std::ifstream _in;
		std::vector<char> _buf;
		double _size = 0, _pos = 0;
	};

	// ---------------------------------------------------------------- PLY
	class PlySource : public PointSource {
		enum class Type { I8, U8, I16, U16, I32, U32, F32, F64 };
		struct Prop { std::string name; Type type; size_t size; size_t offset; };

		static std::string trim(std::string s) {
			while (!s.empty() && (s.back() == '\r' || s.back() == ' ' || s.back() == '\t')) s.pop_back();
			return s;
		}
		static bool parse_type(const std::string& s, Type& t, size_t& n) {
			if (s == "char" || s == "int8") { t = Type::I8;  n = 1; }
			else if (s == "uchar" || s == "uint8") { t = Type::U8;  n = 1; }
			else if (s == "short" || s == "int16") { t = Type::I16; n = 2; }
			else if (s == "ushort" || s == "uint16") { t = Type::U16; n = 2; }
			else if (s == "int" || s == "int32") { t = Type::I32; n = 4; }
			else if (s == "uint" || s == "uint32") { t = Type::U32; n = 4; }
			else if (s == "float" || s == "float32") { t = Type::F32; n = 4; }
			else if (s == "double" || s == "float64") { t = Type::F64; n = 8; }
			else return false;
			return true;
		}
		template <class T> static double load(const char* p) { T v; std::memcpy(&v, p, sizeof(T)); return static_cast<double>(v); }
		static double to_double(const char* p, Type t) {   // setzt Little-Endian-Host voraus (Windows x64)
			switch (t) {
			case Type::I8:  return load<int8_t>(p);
			case Type::U8:  return load<uint8_t>(p);
			case Type::I16: return load<int16_t>(p);
			case Type::U16: return load<uint16_t>(p);
			case Type::I32: return load<int32_t>(p);
			case Type::U32: return load<uint32_t>(p);
			case Type::F32: return load<float>(p);
			case Type::F64: return load<double>(p);
			}
			return 0.0;
		}
		static unsigned char to_byte(double v, Type t) {
			if (t == Type::F32 || t == Type::F64) v *= 255.0;   // 0..1
			else if (t == Type::U16) v /= 257.0;                // 0..65535
			return static_cast<unsigned char>(std::clamp(v + 0.5, 0.0, 255.0));
		}
		int find(std::initializer_list<const char*> names) const {
			for (const char* n : names)
				for (size_t i = 0; i < _props.size(); ++i)
					if (_props[i].name == n) return static_cast<int>(i);
			return -1;
		}

	public:
		bool open(const std::string& path, std::string& error) override {
			_in.open(path, std::ios::binary);
			if (!_in) { error = "cannot open file: " + path; return false; }

			std::string line;
			if (!std::getline(_in, line) || trim(line) != "ply") { error = "not a ply file: " + path; return false; }

			bool in_vertex = false, vertex_seen = false, header_done = false;
			while (std::getline(_in, line)) {
				std::istringstream ss(trim(line));
				std::string kw; ss >> kw;
				if (kw == "format") {
					std::string fmt; ss >> fmt;
					if (fmt == "ascii") _ascii = true;
					else if (fmt == "binary_little_endian") _ascii = false;
					else { error = "unsupported ply format: " + fmt + " (supported: ascii, binary_little_endian)"; return false; }
				}
				else if (kw == "element") {
					std::string name; long long count = 0; ss >> name >> count;
					in_vertex = (name == "vertex");
					if (in_vertex) { vertex_seen = true; _count = count; }
					else if (!vertex_seen) { error = "unsupported ply: element '" + name + "' before 'vertex'"; return false; }
				}
				else if (kw == "property" && in_vertex) {
					std::string type, name; ss >> type;
					if (type == "list") { error = "unsupported ply: list property in vertex element"; return false; }
					ss >> name;
					Type t; size_t n;
					if (!parse_type(type, t, n)) { error = "unsupported ply property type: " + type; return false; }
					_props.push_back({ name, t, n, _rec_size });
					_rec_size += n;
				}
				else if (kw == "end_header") { header_done = true; break; }
			}
			if (!header_done || !vertex_seen || _props.empty()) { error = "invalid ply header: " + path; return false; }

			_ix = find({ "x" }); _iy = find({ "y" }); _iz = find({ "z" });
			_ir = find({ "red", "r" }); _ig = find({ "green", "g" }); _ib = find({ "blue", "b" });
			if (_ix < 0 || _iy < 0 || _iz < 0) { error = "ply has no x/y/z properties"; return false; }
			_has_color = (_ir >= 0 && _ig >= 0 && _ib >= 0);
			return true;
		}

		size_t read(std::vector<PointRecord>& out, size_t max_points) override {
			out.clear();
			if (_read >= _count) return 0;
			const size_t want = static_cast<size_t>(std::min<long long>(static_cast<long long>(max_points), _count - _read));
			if (_ascii) return read_ascii(out, want);

			_buf.resize(want * _rec_size);
			_in.read(_buf.data(), static_cast<std::streamsize>(_buf.size()));
			const size_t n = static_cast<size_t>(_in.gcount()) / _rec_size;
			out.resize(n);
			for (size_t i = 0; i < n; ++i) {
				const char* p = _buf.data() + i * _rec_size;
				PointRecord& r = out[i];
				r.x = to_double(p + _props[_ix].offset, _props[_ix].type);
				r.y = to_double(p + _props[_iy].offset, _props[_iy].type);
				r.z = to_double(p + _props[_iz].offset, _props[_iz].type);
				if (_has_color) {
					r.r = to_byte(to_double(p + _props[_ir].offset, _props[_ir].type), _props[_ir].type);
					r.g = to_byte(to_double(p + _props[_ig].offset, _props[_ig].type), _props[_ig].type);
					r.b = to_byte(to_double(p + _props[_ib].offset, _props[_ib].type), _props[_ib].type);
				}
				else { r.r = r.g = r.b = 128; }
			}
			_read += static_cast<long long>(n);
			return n;
		}
		double progress() const override { return _count > 0 ? std::min(1.0, static_cast<double>(_read) / _count) : 0.0; }

	private:
		size_t read_ascii(std::vector<PointRecord>& out, size_t want) {
			std::string line;
			std::vector<double> v(_props.size());
			while (out.size() < want && std::getline(_in, line)) {
				const char* s = line.c_str();
				char* e = nullptr;
				bool ok = true;
				for (size_t k = 0; k < v.size(); ++k) {
					v[k] = std::strtod(s, &e);
					if (e == s) { ok = false; break; }
					s = e;
				}
				if (!ok) continue;                       // leere/defekte Zeile überspringen
				PointRecord r;
				r.x = v[_ix]; r.y = v[_iy]; r.z = v[_iz];
				if (_has_color) {
					r.r = to_byte(v[_ir], _props[_ir].type);
					r.g = to_byte(v[_ig], _props[_ig].type);
					r.b = to_byte(v[_ib], _props[_ib].type);
				}
				else { r.r = r.g = r.b = 128; }
				out.push_back(r);
			}
			_read += static_cast<long long>(out.size());
			return out.size();
		}

		std::ifstream _in;
		std::vector<char> _buf;
		std::vector<Prop> _props;
		size_t _rec_size = 0;
		bool _ascii = false, _has_color = false;
		int _ix = -1, _iy = -1, _iz = -1, _ir = -1, _ig = -1, _ib = -1;
		long long _count = 0, _read = 0;
	};

#ifdef GIRAFFE_WITH_LASZIP
	// ---------------------------------------------------------------- LAS / LAZ (LASzip)
	class LasSource : public PointSource {
	public:
		~LasSource() override {
			if (_reader) { laszip_close_reader(_reader); laszip_destroy(_reader); }
		}
		bool open(const std::string& path, std::string& error) override {
			if (laszip_create(&_reader)) { error = "laszip_create failed"; return false; }
			laszip_BOOL compressed = 0;
			if (laszip_open_reader(_reader, path.c_str(), &compressed)) {
				laszip_CHAR* msg = nullptr;
				laszip_get_error(_reader, &msg);
				error = std::string("cannot open las/laz file: ") + (msg ? msg : path.c_str());
				return false;
			}
			laszip_header* h = nullptr;
			laszip_get_header_pointer(_reader, &h);
			_count = h->number_of_point_records ? static_cast<long long>(h->number_of_point_records)
				: static_cast<long long>(h->extended_number_of_point_records);
			const unsigned f = h->point_data_format;
			_has_rgb = (f == 2 || f == 3 || f == 5 || f == 7 || f == 8 || f == 10);
			laszip_get_point_pointer(_reader, &_point);
			return true;
		}
		size_t read(std::vector<PointRecord>& out, size_t max_points) override {
			out.clear();
			out.reserve(max_points);
			_raw.clear();
			double xyz[3];
			while (out.size() < max_points && _read < _count) {
				if (laszip_read_point(_reader)) break;
				laszip_get_coordinates(_reader, xyz);          // Scale/Offset bereits angewendet
				out.push_back({ xyz[0], xyz[1], xyz[2], 128, 128, 128 });
				if (_has_rgb) { _raw.push_back(_point->rgb[0]); _raw.push_back(_point->rgb[1]); _raw.push_back(_point->rgb[2]); }
				++_read;
			}
			if (_has_rgb && !out.empty()) {
				if (!_depth_known) {                           // 8 oder 16 Bit? am ersten Block entscheiden
					const unsigned short mx = *std::max_element(_raw.begin(), _raw.end());
					_rgb_shift = (mx > 255) ? 8 : 0;
					_depth_known = true;
				}
				for (size_t i = 0; i < out.size(); ++i) {
					out[i].r = static_cast<unsigned char>(_raw[3 * i + 0] >> _rgb_shift);
					out[i].g = static_cast<unsigned char>(_raw[3 * i + 1] >> _rgb_shift);
					out[i].b = static_cast<unsigned char>(_raw[3 * i + 2] >> _rgb_shift);
				}
			}
			return out.size();
		}
		double progress() const override { return _count > 0 ? std::min(1.0, static_cast<double>(_read) / _count) : 0.0; }
	private:
		laszip_POINTER _reader = nullptr;
		laszip_point* _point = nullptr;
		std::vector<unsigned short> _raw;
		long long _count = 0, _read = 0;
		bool _has_rgb = false, _depth_known = false;
		int _rgb_shift = 0;
	};
#endif

} // namespace

bool is_supported_point_cloud(const std::string& path) {
	const std::string e = lower_ext(path);
	if (e == ".pw" || e == ".ply") return true;
#ifdef GIRAFFE_WITH_LASZIP
	if (e == ".las" || e == ".laz") return true;
#endif
	return false;
}

std::unique_ptr<PointSource> make_point_source(const std::string& path, std::string& error) {
	const std::string e = lower_ext(path);
	std::unique_ptr<PointSource> src;
	if (e == ".pw")       src = std::make_unique<PwSource>();
	else if (e == ".ply") src = std::make_unique<PlySource>();
#ifdef GIRAFFE_WITH_LASZIP
	else if (e == ".las" || e == ".laz") src = std::make_unique<LasSource>();
#endif
	else {
		error = "unsupported point cloud format '" + e + "' (supported: .pw, .ply"
#ifdef GIRAFFE_WITH_LASZIP
			+ std::string(", .las, .laz")
#endif
			+ ")";
		return nullptr;
	}
	if (!src->open(path, error)) return nullptr;
	return src;
}