//
// Created by ilya on 15.07.2021.
//

#include "V4l2Camera.hpp"
#include <stdexcept>
#include <cassert>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <algorithm>
#include <unistd.h>

V4l2Camera::MmapedV4l2Buffer::MmapedV4l2Buffer(const std::byte *pData, const uint32_t dataSize, const uint32_t width,
                                               const uint32_t height, const uint32_t bytesPerLine,
                                               const uint32_t pixFormat) :
    p_data_(pData), data_size_(dataSize),
    width_(width), height_(height),
    bytes_per_line_(bytesPerLine),
    pix_format_(pixFormat) {}

V4l2Camera::MmapedV4l2Buffer::~MmapedV4l2Buffer() {
  if (p_data_) {
    munmap((void *) p_data_, data_size_);
  }
}

void V4l2Camera::openDevice() {
  if (fd_!=-1) {
    assert("Camera already opened");
  }

  fd_ = ::open(dev_path_.c_str(), O_RDWR, 0);
  if (fd_==-1)
    throw std::runtime_error(("Can't openDevice " + dev_path_));
}

void V4l2Camera::closeDevice() const {
  if (fd_!=-1) {
    ::close(fd_);
  }
}

void V4l2Camera::checkCapabilities() const {
  if (fd_==-1) {
    assert("Camera not opened");
  }

  v4l2_capability cap{};
  if (ioctl(fd_, VIDIOC_QUERYCAP, &cap))
    throw std::runtime_error("Fail ioctl VIDIOC_QUERYCAP: " + std::to_string(errno));

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    throw std::runtime_error("Device is not a video capture device");

  if (!(cap.capabilities & V4L2_CAP_STREAMING))
    throw std::runtime_error("Device does not support streaming");
}

#ifdef WITH_PRINTING

#include <iostream>
#include <boost/format.hpp>
#include <utility>

[[maybe_unused]] void V4l2Camera::printDevCapabilities() const {
  if (fd_==-1)
    throw std::runtime_error("Device is not opened");

  v4l2_capability cap{};
  if (ioctl(fd_, VIDIOC_QUERYCAP, &cap))
    throw std::runtime_error("Fail ioctl VIDIOC_QUERYCAP: " + std::to_string(errno));

  std::cout << boost::format{"Opened %s: \n"}%dev_path_
            << boost::format{"\tdriver : %s\n"}%cap.driver
            << boost::format{"\tcard : %s\n"}%cap.card
            << boost::format{"\tbus_info : %s\n"}%cap.bus_info
            << boost::format{"\tversion : %d.%d.%d\n"}
                %((cap.version >> 16) & 0xff)%((cap.version >> 8) & 0xff)%((cap.version >> 0) & 0xff)
            << boost::format{"\tcapabilities : %#x\n"}%cap.capabilities
            << boost::format{"\tdevice capabilities : %#x\n"}%cap.device_caps;
}

[[maybe_unused]] void V4l2Camera::printSupportedImageFormats() const {
  if (fd_==-1)
    throw std::runtime_error("Device is not opened");

  struct v4l2_fmtdesc desc{};
  desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  std::cout << "\t-----\n\tCamera Formats:\n";
  for (; ioctl(fd_, VIDIOC_ENUM_FMT, &desc)==0; desc.index++) {
    char c = desc.flags & V4L2_FMT_FLAG_COMPRESSED ? 'C' : ' ';
    char e = desc.flags & V4L2_FMT_FLAG_EMULATED ? 'E' : ' ';

    std::cout << boost::format{"\t%c%c %s\n"}%c%e%desc.description;

    v4l2_frmsizeenum frmsizeenum{};
    frmsizeenum.pixel_format = desc.pixelformat;
    for (; ioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &frmsizeenum)==0; frmsizeenum.index++) {
      if (frmsizeenum.type==V4L2_FRMSIZE_TYPE_DISCRETE) {
        std::cout << boost::format{"\t\t\t%dx%d FPS: "}%frmsizeenum.discrete.width%
            frmsizeenum.discrete.height;

        v4l2_frmivalenum frmivalenum{};
        frmivalenum.pixel_format = desc.pixelformat;
        frmivalenum.width = frmsizeenum.discrete.width;
        frmivalenum.height = frmsizeenum.discrete.height;
        for (; ioctl(fd_, VIDIOC_ENUM_FRAMEINTERVALS, &frmivalenum)==0; frmivalenum.index++) {
          std::cout
              << static_cast<float>(frmivalenum.discrete.denominator)/static_cast<float>(frmivalenum.discrete.numerator)
              << ' ';
        }

        std::cout << std::endl;
      } else {
        std::cerr << "Unsupported frmsizeenum type: " << frmsizeenum.type << std::endl;
      }
    }
  }
}

[[maybe_unused]] void V4l2Camera::printCropCapabilities() const {
  if (fd_==-1)
    throw std::runtime_error("Device is not opened");

  v4l2_cropcap cap{};
  cap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_CROPCAP, &cap)) {
    if (errno==ENODATA)
      std::cerr << "Cropping is not supported for this input or output\n";
    else if (errno==EINVAL)
      std::cerr << "v4l2_cropcap struct is invalid\n";
    else if (errno==ENOTTY)
      std::cerr
          << "The ioctl is not supported by the driver (the required functionality is not available) or the file descriptor is not for a media device\n";
    else
      std::cerr << "Fail ioctl VIDIOC_CROPCAP: " << errno << std::endl;
    return;
  }

  std::cout << "Camera Cropping:\n"
            << boost::format{"\tBounds : %dx%d+%d+%d\n"}
                %cap.bounds.width%cap.bounds.height%cap.bounds.left%cap.bounds.top
            << boost::format{"\tDefault: %dx%d+%d+%d\n"}
                %cap.defrect.width%cap.defrect.height%cap.defrect.left%cap.defrect.top
            << boost::format{"\tAspect: %d/%d\n"}%cap.pixelaspect.numerator%cap.pixelaspect.denominator;
}

#endif

V4l2Camera::img_params_t V4l2Camera::getImageParams() const {
  struct v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_G_FMT, &fmt))
    throw std::runtime_error("Fail ioctl VIDIOC_G_FMT: " + std::to_string(errno));

  return V4l2Camera::img_params_t{
      fmt.fmt.pix.sizeimage,
      fmt.fmt.pix.width,
      fmt.fmt.pix.height,
      fmt.fmt.pix.bytesperline,
      fmt.fmt.pix.pixelformat
  };
}

void V4l2Camera::initFrames(uint buffers_count) {
  if (!frames_.empty()) {
    assert("Frames not empty");
  }

  requestBufferAllocationFromDriver(buffers_count);
  const auto image_params = getImageParams();
  mmapFramesBufferAllocatedByDriver(buffers_count, image_params);
}

void V4l2Camera::cleanFrames() {
  if (frames_.empty())
    return;

  frames_.clear();

  v4l2_requestbuffers rq_bufs{};
  constexpr auto FREE_ALL_DEVICE_BUFFERS = 0;
  rq_bufs.count = FREE_ALL_DEVICE_BUFFERS;
  rq_bufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rq_bufs.memory = V4L2_MEMORY_MMAP;
  ioctl(fd_, VIDIOC_REQBUFS, &rq_bufs);
}

void V4l2Camera::open() {
  if (!frames_.empty())
    throw std::runtime_error("Already opened");

  openDevice();
  checkCapabilities();
  updateCtrlParametersVector();
}

void V4l2Camera::startCapturing(uint buffers_count) {
  if (buffers_count==0) {
    assert("buffers_count can't be 0");
  }

  if (fd_==-1)
    throw std::runtime_error("Needs to be opened first");

  if (!frames_.empty())
    throw std::runtime_error("Capturing is already started");

  initFrames(buffers_count);
  pushFramesToDriverQueue();
  enableCapturing();
}

void V4l2Camera::stopCapturing() {
  if (!frames_.empty()) {
    disableCapturing();
    cleanFrames();
  }
}

void V4l2Camera::close() {
  stopCapturing();
  closeDevice();
}

V4l2Camera::V4l2Camera(std::string devPath) : frames_{}, dev_path_(std::move(devPath)), fd_(-1) {}

V4l2Camera::~V4l2Camera() {
  stopCapturing();
  close();
}

const V4l2Camera::MmapedV4l2Buffer &V4l2Camera::getNewImage() {
  v4l2_buffer buf{};

  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd_, VIDIOC_DQBUF, &buf))
    throw std::runtime_error("Fail ioctl VIDIOC_DQBUF: " + std::to_string(errno));

  return frames_[buf.index];
}

[[maybe_unused]] const std::string &V4l2Camera::getDevPath() const {
  return dev_path_;
}

[[maybe_unused]] std::vector<V4l2Camera::mode_t> V4l2Camera::getSensorModes() const {
  std::vector<V4l2Camera::mode_t> modes_vector{};
  modes_vector.reserve(8);

  int camera_mode_index = 0;

  struct v4l2_fmtdesc desc{};
  desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  for (; ioctl(fd_, VIDIOC_ENUM_FMT, &desc)==0; desc.index++) {
    bool flag_compressed = desc.flags & V4L2_FMT_FLAG_COMPRESSED;
    bool flag_emulated = desc.flags & V4L2_FMT_FLAG_EMULATED;

    v4l2_frmsizeenum frmsizeenum{};
    frmsizeenum.pixel_format = desc.pixelformat;
    for (; ioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &frmsizeenum)==0; frmsizeenum.index++) {
      if (frmsizeenum.type==V4L2_FRMSIZE_TYPE_DISCRETE) {
        v4l2_frmivalenum frmivalenum{};
        frmivalenum.pixel_format = desc.pixelformat;
        frmivalenum.width = frmsizeenum.discrete.width;
        frmivalenum.height = frmsizeenum.discrete.height;
        for (; ioctl(fd_, VIDIOC_ENUM_FRAMEINTERVALS, &frmivalenum)==0; frmivalenum.index++) {
          modes_vector.emplace_back(desc.pixelformat,
                                    frmsizeenum.discrete.width,
                                    frmsizeenum.discrete.height,
                                    camera_mode_index++,
                                    static_cast<float>(frmivalenum.discrete.denominator)
                                        /static_cast<float>(frmivalenum.discrete.numerator),
                                    flag_compressed,
                                    flag_emulated);
        }
      } else {
        throw std::runtime_error("Unsupported frmsizeenum type: " + std::to_string(frmsizeenum.type));
      }
    }
  }
  return modes_vector;
}

constexpr char SENSOR_MODE_STRING[] = "sensor mode";
void V4l2Camera::setSensorMode(const mode_t &m) {
  setV4l2Ctrl<SENSOR_MODE_STRING>(m.camera_mode_index);

  struct v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_G_FMT, &fmt))
    throw std::runtime_error("Fail ioctl VIDIOC_G_FMT: " + std::to_string(errno));

  fmt.fmt.pix.height = m.height;
  fmt.fmt.pix.width = m.width;
  fmt.fmt.pix.pixelformat = m.pix_format;

  if (ioctl(fd_, VIDIOC_S_FMT, &fmt))
    throw std::runtime_error("Fail ioctl VIDIOC_G_FMT: " + std::to_string(errno));

  updateCtrlParametersVector();
}

void V4l2Camera::mmapFramesBufferAllocatedByDriver(uint buffers_count, const img_params_t &img_params) {
  for (uint i = 0; i < buffers_count; i++) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(fd_, VIDIOC_QUERYBUF, &buf))
      throw std::runtime_error("Fail ioctl VIDIOC_QUERYBUF: " + std::to_string(errno));

    void *data = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
    if (data==MAP_FAILED)
      throw std::runtime_error("Could not mmap buffer");

    frames_.emplace_back((std::byte *) data,
                         buf.length,
                         img_params.width,
                         img_params.height,
                         img_params.bytes_per_line,
                         img_params.pix_format);
  }
}

void V4l2Camera::requestBufferAllocationFromDriver(uint buffers_count) {
  v4l2_requestbuffers rq_bufs{};

  rq_bufs.count = buffers_count;
  rq_bufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rq_bufs.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd_, VIDIOC_REQBUFS, &rq_bufs))
    throw std::runtime_error("Fail ioctl VIDIOC_REQBUFS: " + std::to_string(errno));

  if (rq_bufs.count <= 0)
    throw std::runtime_error("Can't allocate requested buffers");

  if (rq_bufs.count!=buffers_count) {
    // DEALLOCATE BUFFERS
    cleanFrames();
    throw std::runtime_error("Allocated only " + std::to_string(rq_bufs.count) + "buffers, but " +
        std::to_string(buffers_count) + "requested (deallocated)");
  }
}

void V4l2Camera::pushFramesToDriverQueue() {
  for (int i = 0; i < frames_.size(); ++i) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(fd_, VIDIOC_QBUF, &buf))
      throw std::runtime_error("Fail ioctl VIDIOC_QBUF: " + std::to_string(errno));
  }
}

void V4l2Camera::enableCapturing() const {
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_STREAMON, &type))
    throw std::runtime_error("Could not startCapturing streaming");
}

void V4l2Camera::disableCapturing() const {
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_STREAMOFF, &type))
    throw std::runtime_error("Could not startCapturing streaming");
}

void V4l2Camera::updateCtrlParametersVector() {
  ctrl_parameters_.clear();

  struct v4l2_query_ext_ctrl query_ext_ctrl = {};

  query_ext_ctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
  while (0==ioctl(fd_, VIDIOC_QUERY_EXT_CTRL, &query_ext_ctrl)) {
    if (!(query_ext_ctrl.flags & (V4L2_CTRL_FLAG_DISABLED | V4L2_CTRL_FLAG_READ_ONLY))) {
      std::string tmp = query_ext_ctrl.name;
      std::transform(std::begin(tmp),
                     std::end(tmp),
                     std::begin(tmp),
                     [](char c) { return std::tolower(c); });

      ctrl_parameters_.emplace_back(tmp,
                                    query_ext_ctrl.id,
                                    query_ext_ctrl.minimum,
                                    query_ext_ctrl.maximum,
                                    query_ext_ctrl.default_value);
    }

    query_ext_ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  }

  if (errno!=EINVAL) {
    throw std::runtime_error("VIDIOC_QUERY_EXT_CTRL errno: " + std::to_string(errno));
  }
}

[[maybe_unused]] const std::vector<V4l2Camera::CtrlParameter> &V4l2Camera::getControlParameters() {
  return ctrl_parameters_;
}

V4l2Camera::mode_t::mode_t(const uint32_t pix_format,
                           const uint_fast16_t width,
                           const uint_fast16_t height,
                           const uint32_t camera_mode_index,
                           const float frame_rate,
                           bool compressed,
                           bool emulated)
    : pix_format(pix_format),
      width(width),
      height(height),
      camera_mode_index(camera_mode_index),
      frame_rate(frame_rate),
      compressed(compressed),
      emulated(emulated) {}

V4l2Camera::CtrlParameter::CtrlParameter(std::string name,
                                         const uint32_t v_4_l_2_id,
                                         const uint32_t minimum_val,
                                         const uint32_t maximum_val,
                                         const uint32_t default_val)
    : name(std::move(name)),
      v4l2_id(v_4_l_2_id),
      minimum_val(minimum_val),
      maximum_val(maximum_val),
      default_val(default_val) {}

V4l2Camera::V4l2CtlVal::V4l2CtlVal(const uint32_t current_val,
                                   const uint32_t minimum_val,
                                   const uint32_t maximum_val,
                                   const uint32_t default_val)
    : current_val(current_val), minimum_val(minimum_val), maximum_val(maximum_val), default_val(default_val) {}

template<char const *ctl_name>
V4l2Camera::V4l2CtlVal V4l2Camera::setV4l2Ctrl(int value) {
  auto v4l2_ctl =
      *std::find_if(std::cbegin(ctrl_parameters_), std::cend(ctrl_parameters_), [](const CtrlParameter &p) {
        return p.name==ctl_name;
      });

  struct v4l2_ext_control extCtrl[1] = {{}};
  struct v4l2_ext_controls ext_ctrls{};

  extCtrl[0].id = v4l2_ctl.v4l2_id;
  extCtrl[0].size = 0;
  extCtrl[0].value = value;
  ext_ctrls.controls = extCtrl;
  ext_ctrls.count = 1;
  ext_ctrls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;

  if (ioctl(fd_, VIDIOC_S_EXT_CTRLS, &ext_ctrls))
    throw std::runtime_error("Fail ioctl VIDIOC_S_EXT_CTRLS: " + std::to_string(errno));

  return V4l2Camera::Gain(
      ext_ctrls.controls->value, v4l2_ctl.minimum_val, v4l2_ctl.maximum_val, v4l2_ctl.default_val);
}

template<char const *ctl_name>
V4l2Camera::V4l2CtlVal V4l2Camera::getV4l2Ctrl() {
  auto v4l2_ctl =
      *std::find_if(std::cbegin(ctrl_parameters_), std::cend(ctrl_parameters_), [](const CtrlParameter &p) {
        return p.name==ctl_name;
      });

  struct v4l2_ext_control extCtrl[1] = {{}};
  struct v4l2_ext_controls ext_ctrls{};

  extCtrl[0].id = v4l2_ctl.v4l2_id;
  extCtrl[0].size = 0;
  ext_ctrls.controls = extCtrl;
  ext_ctrls.count = 1;
  ext_ctrls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;

  if (ioctl(fd_, VIDIOC_G_EXT_CTRLS, &ext_ctrls))
    throw std::runtime_error("Fail ioctl VIDIOC_G_EXT_CTRLS: " + std::to_string(errno));

  return V4l2Camera::Gain(
      ext_ctrls.controls->value, v4l2_ctl.minimum_val, v4l2_ctl.maximum_val, v4l2_ctl.default_val);
}

constexpr char GAIN_STRING[] = "gain";
V4l2Camera::Gain V4l2Camera::getGain() {
  return getV4l2Ctrl<GAIN_STRING>();
}

V4l2Camera::Gain V4l2Camera::setGain(uint32_t val) {
  return setV4l2Ctrl<GAIN_STRING>(val);
}

constexpr char EXPOSURE_STRING[] = "exposure";
V4l2Camera::Exposure V4l2Camera::getExposure() {
  return getV4l2Ctrl<EXPOSURE_STRING>();
}

V4l2Camera::Exposure V4l2Camera::setExposure(uint32_t val) {
  return setV4l2Ctrl<EXPOSURE_STRING>(val);
}
