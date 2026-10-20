#pragma once
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

struct PointRecord {
	double x, y, z;              // Originalkoordinaten (global, noch ohne shift)
	unsigned char r, g, b;
};

class PointSource {
public:
	virtual ~PointSource() = default;
	virtual bool open(const std::string& path, std::string& error) = 0;
	// füllt 'out' (wird vorher geleert) mit bis zu max_points Punkten; Rückgabe 0 = Ende
	virtual size_t read(std::vector<PointRecord>& out, size_t max_points) = 0;
	virtual double progress() const = 0;   // 0..1
};

// Wählt den Reader über die Dateiendung (.pw, .ply, .las, .laz) und öffnet die Datei
std::unique_ptr<PointSource> make_point_source(const std::string& path, std::string& error);
bool is_supported_point_cloud(const std::string& path);