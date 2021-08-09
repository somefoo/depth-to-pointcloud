#include <ImfFrameBuffer.h>
#include <ImfPixelType.h>
#include <OpenEXR/ImfRgbaFile.h>
#include <OpenEXR/OpenEXRConfig.h>

#include <cstddef>
#include <iostream>
#include <ostream>
#include <fstream>
//#include <OpenEXR/ImfArray.h>
//#include <OpenEXR/ImfNamespace.h>
#include <OpenEXR/ImfArray.h>
#include <OpenEXR/ImfInputFile.h>

#include <limits>

//static float upper_cut = std::numeric_limits<float>::max();
static float upper_cut = 65500;
static float lower_cut = std::numeric_limits<float>::min();

bool is_valid(float value){
  return value < upper_cut && value > lower_cut;
}

std::size_t count_valid_points(const Imf::Array2D<float> &depth_pixels){
  std::size_t valid_counter = 0;
  for (std::size_t y = 0; y < depth_pixels.height(); ++y) {
    for (std::size_t x = 0; x < depth_pixels.width(); ++x) {
      if(is_valid(depth_pixels[y][x])) valid_counter++;
    }
  }
  return valid_counter;
}

void print_detail_ascii(const Imf::Array2D<float> &depth_pixels) {
  float color = depth_pixels[0][0];
  for (std::size_t y = 0; y < depth_pixels.height(); ++y) {
    for (std::size_t x = 0; x < depth_pixels.width(); ++x) {
      if (depth_pixels[y][x] != color) {
        std::cout << "#";
      }else{
        std::cout << " ";
      }
    }
    std::cout << '\n';
  }
}

void sanity_check(const Imf::Array2D<float> &depth_pixels) {
  float color = depth_pixels[0][0];
  for (std::size_t y = 0; y < depth_pixels.height(); ++y) {
    for (std::size_t x = 0; x < depth_pixels.width(); ++x) {
      if (depth_pixels[y][x] != color) {
        return;
      }
    }
  }
  std::cerr << "Error: depth image is homogenous, exiting.\n";
  exit(1);
}

// See http://pointclouds.org/documentation/tutorials/pcd_file_format.html
void print_pcd(const Imf::Array2D<float> &depth_pixels, std::ostream& ostream = std::cout) {
  std::size_t valid_count = count_valid_points(depth_pixels);
  ostream << "# .PCD v.7 - Point Cloud Data file format\n";
  ostream << "VERSION .7\n";
  ostream << "FIELDS x y z rgb\n";
  ostream << "SIZE 4 4 4 4\n";
  ostream << "TYPE F F F F\n";
  ostream << "COUNT 1 1 1 1\n";
  ostream << "WIDTH " << valid_count << '\n';
  ostream << "HEIGHT 1\n";
  ostream << "VIEWPOINT 0 0 0 1 0 0 0\n";
  ostream << "POINTS " << valid_count << '\n';
  ostream << "DATA ascii\n";
  for (std::size_t y = 0; y < depth_pixels.height(); ++y) {
    for (std::size_t x = 0; x < depth_pixels.width(); ++x) {
      if(is_valid(depth_pixels[y][x])){
        ostream << x << " " << y << " " << depth_pixels[y][x] * 2 << " " << 4.2108e+06 << '\n';
      }
    }
  }
  ostream.flush();
}

int main(int argc, char *argv[]) {
  // TODO add checks
  Imf::InputFile file(argv[1]);
  Imath::Box2i dim = file.header().dataWindow();

  const int width = dim.max.x - dim.min.x + 1;
  const int height = dim.max.y - dim.min.y + 1;
  std::cout << "Image WxH: " << width << "x" << height << '\n';

  Imf::Array2D<float> depth_pixels(height, width);
  const auto base_ptr =
      (char *) (&depth_pixels[0][0] - dim.min.x - dim.min.y * width);
  const std::size_t x_stride = sizeof(depth_pixels[0][0]);
  const std::size_t y_stride = sizeof(depth_pixels[0][0]) * width;

  std::cout << x_stride << " " << y_stride << "\n";
 
  Imf::FrameBuffer frame_buffer;
  frame_buffer.insert("Z", Imf::Slice(Imf::FLOAT, base_ptr, x_stride, y_stride,
                                      1, 1, std::numeric_limits<float>::max()));

  file.setFrameBuffer(frame_buffer);
  file.readPixels(dim.min.y, dim.max.y);
  sanity_check(depth_pixels);

  std::ofstream stream ("new.pcd", std::ofstream::binary);
  print_pcd(depth_pixels, stream);
}
