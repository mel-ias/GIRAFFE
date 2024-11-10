# GIRAFFE -  Geospatial Image Registration And reFErencing

![GIRAFFE - Logo](https://github.com/mel-ias/GIRAFFE/blob/I2G_publishing/GIRAFFE/docs/logo/banner.png?raw=true)

**GIRAFFE** is an open-source tool designed for precise registration and orientation of 2D images within 3D geometries, specifically point clouds. It enables image projection and alignment within the coordinate space of the point cloud, supporting workflows like visualization, georeferencing, and coloring.

## Features
- Image-to-Geometry Registration: Aligns images within a point cloud and links image data with geometric features.
- Image Rendering from Point Clouds: Generates synthetic views from point clouds to support image orientation.
- Feature Extraction and Matching: Detects and extracts image and geometry features for accurate spatial registration.
- Frustum Culling: Optimizes visibility and rendering by calculating a local frustum and transforming it into world coordinates.
- Automatic Calibration: Dynamically adjusts calibration parameters based on the distribution of matched points.

*Note: GIRAFFE includes the header-only library "JSON for Modern C++" (https://github.com/nlohmann/json). We would like to take the opportunity and thank the authors for providing this incredible useful library.*


## Installation

1. Clone this repository:
```bash
git clone https://github.com/mel-ias/GIRAFFE.git --branch I2G_publishing
cd GIRAFFE
```
2. Dependencies: Install required dependencies as listed in the CMakeLists.txt file.
3. Build with CMake (tested with [MSVC Compiler using Buildtools für Visual Studio 2022](https://visualstudio.microsoft.com/de/downloads/#build-tools-for-visual-studio-2022 "MSVC Compiler using Buildtools für Visual Studio 2022")):

```bash
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

#### Integrate Lightglue for true-synthetic image matching
Create a new virtual python environemnt as prerequisite to run lightglue image matching in the cloned project directory and follow the installation instruction from lightglue (https://github.com/cvg/LightGlue).  We recommend to install Lightglue in a virtual Python 3.10. environment:

```bash
python -m venv .venv
.venv\Scripts\activate.bat
git clone https://github.com/cvg/LightGlue.git && cd LightGlue
python -m pip install .
```

#### Copy initialisation file
GIRAFFE.exe requires a few initialisation parameters that are preconfigured in the given init.json file that has to be copied to the directory of GIRAFFE.(.exe). More details on the settings are given below.

## Usage
This example script demonstrates how to set up and execute the GIRAFFE.exe with sample arguments for processing a point cloud and image data.

### Example Batch Script Usage
Here is a sample batch script to configure and run GIRAFFE:

```bash
@echo off
:: Define paths for executable, virtual environment, and arguments
set "VIG_DIR=[...]\x64\VIG_Release_CV_410_x64"
set "VENV_DIR=%VIG_DIR%\.venv"
set "POINT_CLOUD_PATH=[...]\point_cloud.pw" :: PW format mandatory
set "JSON_PATH=[...]\cam\cam_params.json"
set "PYTHON_SCRIPT_PATH=%VIG_DIR%\match_pairs_lightglue.py" :: link to the provided python file that executes lightglue-based matching
set "PROJECT_NAME=my_project" :: specify project name

:: Activate the virtual environment
call "%VENV_DIR%\Scripts\activate.bat"

:: Run GIRAFFE.exe with specified arguments
"%VIG_DIR%\GIRAFFE.exe" -i "%POINT_CLOUD_PATH%" -j "%JSON_PATH%" -p "%PYTHON_SCRIPT_PATH%" -n "%PROJECT_NAME%"

:: Pause to keep the console open
pause
```
#### Argument Details
- `-i`: Path to the point cloud file (e.g., .pw format).
- `-j`: Path to the JSON configuration file containing image or synthetic view data.
- `-p`: Path to the Python script (e.g., match_pairs_lightglue.py) for feature matching.
- `-n`: Project name for the current run, used to organize output data.

## Additional Files
The **init.json** file configures GIRAFFE's initial parameters:
bash:
```
{
  "neighbour_distance": 2.5,
  "final_iteration_number": 3,
  "calc_IO": false,
  "ultra_wide_camera": false,
  "output_pointcloud": true
}
```

- neighbour_distance: Sets the maximum distance to consider neighbors.
- final_iteration_number: Defines the number of final alignment iterations.
- calc_IO: Enables or disables intrinsic calibration.
- ultra_wide_camera: Specifies if the camera is ultra-wide.
- output_pointcloud: Toggles output of the processed point cloud.

The **match_pairs_lightglue.py** python script handles feature matching using the LightGlue library. Key arguments (that are autmatically set and called by GIRAFFE) include:

```python
"--left_image: Path to the real image."
"--right_image: Path to the synthetic image."
"--output_dir: Directory for saving matched keypoints."
```
The script loads and processes images, extracts features, performs matching, and outputs matched keypoints to kpts.txt within the specified output_dir (match_pairs_lightglue).

## Running GIRAFFE
To run the batch script:

Copy the script content into a .bat file.
Update paths as necessary for your setup.
Execute the batch file to start the GIRAFFE process with specified arguments.

Running GIRAFFE
To run GIRAFFE with the batch script:

Copy the script content into a .bat file.
Update paths as necessary.
Execute the batch file to start the GIRAFFE process

Documentation

To generate HTML documentation:

Install Doxygen.
Run:
bash

`
doxygen Doxyfile
`

The HTML documentation will be located in the docs folder.

### Contributing
Contributions are welcome! Please fork the repository and submit a pull request for review.

### License
This project is licensed under the MIT License. See the LICENSE file for details.



How to setup



2. b) build GIRAFFE for your system using the prepared CMakeList.txt file (We recommend Buildtools for Visual Studio 2022 for this job (https://visualstudio.microsoft.com/de/downloads/#build-tools-for-visual-studio-2022)). Download OpenCV 4.10.0 for your system (using either the prebuilt version or built by your own using the OpenCV instructions) and link to your OpenCV installation in the CMakeList.txt file


3. GIRAFFE requires several to be runned. We recommend building a batch / shell script to run the tool without any efforts. An example for this is given below:

@echo off

:: Define the paths for the executable, virtual environment, and arguments
set "VIG_DIR=<path_to_the_dir_of_giraffe.exe" 
set "VENV_DIR=%VIG_DIR%\.venv" -> this requires that the .venv for lightglue is in the same directory as GIRAFFE.exe
set "POINT_CLOUD_PATH=<path_to_pw_file>"
set "JSON_PATH=<path_to_synth_file>"
set "PYTHON_SCRIPT_PATH=%VIG_DIR%\match_pairs_lightglue.py"
set "PROJECT_NAME=<project_name>"

:: Activate the virtual environment
call "%VENV_DIR%\Scripts\activate.bat"

:: Run GIRAFFE.exe with specified arguments
"%VIG_DIR%\GIRAFFE.exe" -i "%POINT_CLOUD_PATH%" -j "%JSON_PATH%" -p "%PYTHON_SCRIPT_PATH%" -n "%PROJECT_NAME%"

:: Pause to keep the console open
pause












### Features

- Support Standard Markdown / CommonMark and GFM(GitHub Flavored Markdown);
- Full-featured: Real-time Preview, Image (cross-domain) upload, Preformatted text/Code blocks/Tables insert, Code fold, Search replace, Read only, Themes, Multi-languages, L18n, HTML entities, Code syntax highlighting...;
- Markdown Extras : Support ToC (Table of Contents), Emoji, Task lists, @Links...;
- Compatible with all major browsers (IE8+), compatible Zepto.js and iPad;
- Support identification, interpretation, fliter of the HTML tags;
- Support TeX (LaTeX expressions, Based on KaTeX), Flowchart and Sequence Diagram of Markdown extended syntax;
- Support AMD/CMD (Require.js & Sea.js) Module Loader, and Custom/define editor plugins;

# Editor.md

![](https://pandao.github.io/editor.md/images/logos/editormd-logo-180x180.png)

![](https://img.shields.io/github/stars/pandao/editor.md.svg) ![](https://img.shields.io/github/forks/pandao/editor.md.svg) ![](https://img.shields.io/github/tag/pandao/editor.md.svg) ![](https://img.shields.io/github/release/pandao/editor.md.svg) ![](https://img.shields.io/github/issues/pandao/editor.md.svg) ![](https://img.shields.io/bower/v/editor.md.svg)


**Table of Contents**

[TOCM]

[TOC]

#H1 header
##H2 header
###H3 header
####H4 header
#####H5 header
######H6 header
#Heading 1 link [Heading link](https://github.com/pandao/editor.md "Heading link")
##Heading 2 link [Heading link](https://github.com/pandao/editor.md "Heading link")
###Heading 3 link [Heading link](https://github.com/pandao/editor.md "Heading link")
####Heading 4 link [Heading link](https://github.com/pandao/editor.md "Heading link") Heading link [Heading link](https://github.com/pandao/editor.md "Heading link")
#####Heading 5 link [Heading link](https://github.com/pandao/editor.md "Heading link")
######Heading 6 link [Heading link](https://github.com/pandao/editor.md "Heading link")

##Headers (Underline)

H1 Header (Underline)
=============

H2 Header (Underline)
-------------

###Characters
                
----

~~Strikethrough~~ <s>Strikethrough (when enable html tag decode.)</s>
*Italic*      _Italic_
**Emphasis**  __Emphasis__
***Emphasis Italic*** ___Emphasis Italic___

Superscript: X<sub>2</sub>，Subscript: O<sup>2</sup>

**Abbreviation(link HTML abbr tag)**

The <abbr title="Hyper Text Markup Language">HTML</abbr> specification is maintained by the <abbr title="World Wide Web Consortium">W3C</abbr>.

###Blockquotes

> Blockquotes

Paragraphs and Line Breaks
                    
> "Blockquotes Blockquotes", [Link](http://localhost/)。

###Links

[Links](http://localhost/)

[Links with title](http://localhost/ "link title")

`<link>` : <https://github.com>

[Reference link][id/name] 

[id/name]: http://link-url/

GFM a-tail link @pandao

###Code Blocks (multi-language) & highlighting

####Inline code

`$ npm install marked`

####Code Blocks (Indented style)

Indented 4 spaces, like `<pre>` (Preformatted Text).

    <?php
        echo "Hello world!";
    ?>
    
Code Blocks (Preformatted text):

    | First Header  | Second Header |
    | ------------- | ------------- |
    | Content Cell  | Content Cell  |
    | Content Cell  | Content Cell  |

####Javascript　

```javascript
function test(){
	console.log("Hello world!");
}
 
(function(){
    var box = function(){
        return box.fn.init();
    };

    box.prototype = box.fn = {
        init : function(){
            console.log('box.init()');

			return this;
        },

		add : function(str){
			alert("add", str);

			return this;
		},

		remove : function(str){
			alert("remove", str);

			return this;
		}
    };
    
    box.fn.init.prototype = box.fn;
    
    window.box =box;
})();

var testBox = box();
testBox.add("jQuery").remove("jQuery");
```

####HTML code

```html
<!DOCTYPE html>
<html>
    <head>
        <mate charest="utf-8" />
        <title>Hello world!</title>
    </head>
    <body>
        <h1>Hello world!</h1>
    </body>
</html>
```

###Images

Image:

![](https://pandao.github.io/editor.md/examples/images/4.jpg)

> Follow your heart.

![](https://pandao.github.io/editor.md/examples/images/8.jpg)

> 图为：厦门白城沙滩 Xiamen

图片加链接 (Image + Link)：

[![](https://pandao.github.io/editor.md/examples/images/7.jpg)](https://pandao.github.io/editor.md/examples/images/7.jpg "李健首张专辑《似水流年》封面")

> 图为：李健首张专辑《似水流年》封面
                
----

###Lists

####Unordered list (-)

- Item A
- Item B
- Item C
     
####Unordered list (*)

* Item A
* Item B
* Item C

####Unordered list (plus sign and nested)
                
+ Item A
+ Item B
    + Item B 1
    + Item B 2
    + Item B 3
+ Item C
    * Item C 1
    * Item C 2
    * Item C 3

####Ordered list
                
1. Item A
2. Item B
3. Item C
                
----
                    
###Tables
                    
First Header  | Second Header
------------- | -------------
Content Cell  | Content Cell
Content Cell  | Content Cell 

| First Header  | Second Header |
| ------------- | ------------- |
| Content Cell  | Content Cell  |
| Content Cell  | Content Cell  |

| Function name | Description                    |
| ------------- | ------------------------------ |
| `help()`      | Display the help window.       |
| `destroy()`   | **Destroy your computer!**     |

| Item      | Value |
| --------- | -----:|
| Computer  | $1600 |
| Phone     |   $12 |
| Pipe      |    $1 |

| Left-Aligned  | Center Aligned  | Right Aligned |
| :------------ |:---------------:| -----:|
| col 3 is      | some wordy text | $1600 |
| col 2 is      | centered        |   $12 |
| zebra stripes | are neat        |    $1 |
                
----

####HTML entities

&copy; &  &uml; &trade; &iexcl; &pound;
&amp; &lt; &gt; &yen; &euro; &reg; &plusmn; &para; &sect; &brvbar; &macr; &laquo; &middot; 

X&sup2; Y&sup3; &frac34; &frac14;  &times;  &divide;   &raquo;

18&ordm;C  &quot;  &apos;

##Escaping for Special Characters

\*literal asterisks\*

##Markdown extras

###GFM task list

- [x] GFM task list 1
- [x] GFM task list 2
- [ ] GFM task list 3
    - [ ] GFM task list 3-1
    - [ ] GFM task list 3-2
    - [ ] GFM task list 3-3
- [ ] GFM task list 4
    - [ ] GFM task list 4-1
    - [ ] GFM task list 4-2

###Emoji mixed :smiley:

> Blockquotes :star:

####GFM task lists & Emoji & fontAwesome icon emoji & editormd logo emoji :editormd-logo-5x:

- [x] :smiley: @mentions, :smiley: #refs, [links](), **formatting**, and <del>tags</del> supported :editormd-logo:;
- [x] list syntax required (any unordered or ordered list supported) :editormd-logo-3x:;
- [x] [ ] :smiley: this is a complete item :smiley:;
- [ ] []this is an incomplete item [test link](#) :fa-star: @pandao; 
- [ ] [ ]this is an incomplete item :fa-star: :fa-gear:;
    - [ ] :smiley: this is an incomplete item [test link](#) :fa-star: :fa-gear:;
    - [ ] :smiley: this is  :fa-star: :fa-gear: an incomplete item [test link](#);
            
###TeX(LaTeX)
   
$$E=mc^2$$

Inline $$E=mc^2$$ Inline，Inline $$E=mc^2$$ Inline。

$$\(\sqrt{3x-1}+(1+x)^2\)$$
                    
$$\sin(\alpha)^{\theta}=\sum_{i=0}^{n}(x^i + \cos(f))$$
                
###FlowChart

```flow
st=>start: Login
op=>operation: Login operation
cond=>condition: Successful Yes or No?
e=>end: To admin

st->op->cond
cond(yes)->e
cond(no)->op
```

###Sequence Diagram
                    
```seq
Andrew->China: Says Hello 
Note right of China: China thinks\nabout it 
China-->Andrew: How are you? 
Andrew->>China: I am good thanks!
```

###End