#include <ImfFrameBuffer.h>
#include <ImfPixelType.h>
#include <OpenEXR/ImfRgbaFile.h>
#include <OpenEXR/OpenEXRConfig.h>

#include <cstddef>
#include <fstream>
#include <iostream>
#include <ostream>
//#include <OpenEXR/ImfArray.h>
//#include <OpenEXR/ImfNamespace.h>
#include <OpenEXR/ImfArray.h>
#include <OpenEXR/ImfInputFile.h>

#include <limits>
#include <random>

// static float upper_cut = std::numeric_limits<float>::max();
static float upper_cut = 65500;
static float lower_cut = std::numeric_limits<float>::min();
static float keep_probability = 0.5;
static float variance = 1;


void print_detail_ascii(const Imf::Array2D<float> &depth_pixels) {
  float color = depth_pixels[0][0];
  for (std::size_t y = 0; y < depth_pixels.height(); ++y) {
    for (std::size_t x = 0; x < depth_pixels.width(); ++x) {
      if (depth_pixels[y][x] != color) {
        std::cout << "#";
      } else {
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
void print_pcd(const Imf::Array2D<float> &depth_pixels,
               std::ostream &ostream = std::cout) {
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> d{0, 1};
  std::uniform_real_distribution<> u{0.0, 1.0};

  std::ostringstream header_stream;
  std::ostringstream point_stream;

  std::size_t point_count = 0;

  for (std::size_t y = 0; y < depth_pixels.height(); ++y) {
    for (std::size_t x = 0; x < depth_pixels.width(); ++x) {
      float value = depth_pixels[y][x];
      bool is_inside = value < upper_cut && value > lower_cut;
      bool is_kept = u(gen) <= keep_probability;

      if (is_inside && is_kept) {
        point_stream << x + d(gen) << " " << y + d(gen) << " " << value*2 + d(gen)
                     << " " << 4.2108e+06 << '\n';
        point_count++;
      }
    }
  }

  header_stream << "# .PCD v.7 - Point Cloud Data file format\n";
  header_stream << "VERSION .7\n";
  header_stream << "FIELDS x y z rgb\n";
  header_stream << "SIZE 4 4 4 4\n";
  header_stream << "TYPE F F F F\n";
  header_stream << "COUNT 1 1 1 1\n";
  header_stream << "WIDTH " << point_count << '\n';
  header_stream << "HEIGHT 1\n";
  header_stream << "VIEWPOINT 0 0 0 1 0 0 0\n";
  header_stream << "POINTS " << point_count << '\n';
  header_stream << "DATA ascii\n";

  ostream << header_stream.str();
  ostream << point_stream.str();
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
      (char *)(&depth_pixels[0][0] - dim.min.x - dim.min.y * width);
  const std::size_t x_stride = sizeof(depth_pixels[0][0]);
  const std::size_t y_stride = sizeof(depth_pixels[0][0]) * width;

  std::cout << x_stride << " " << y_stride << "\n";

  Imf::FrameBuffer frame_buffer;
  frame_buffer.insert("Z", Imf::Slice(Imf::FLOAT, base_ptr, x_stride, y_stride,
                                      1, 1, std::numeric_limits<float>::max()));

  file.setFrameBuffer(frame_buffer);
  file.readPixels(dim.min.y, dim.max.y);
  sanity_check(depth_pixels);

  std::ofstream stream("new.pcd", std::ofstream::binary);
  print_pcd(depth_pixels, stream);
}
