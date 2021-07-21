//
// Created by ilya on 15.07.2021.
//

#ifndef V4L2_EXAMPLE_V4L2CAMERA_HPP
#define V4L2_EXAMPLE_V4L2CAMERA_HPP

#include <string>
#include <vector>
#include <cstdint>

class V4l2Camera {
 public:
  static constexpr uint DEFAULT_BUFFERS_COUNT = 4;

 private:
  class MmapedV4l2Buffer {
    const std::byte *p_data_;
    const uint32_t data_size_;

    const uint32_t width_;
    const uint32_t height_;
    const uint32_t bytes_per_line_;
    const uint32_t pix_format_;

   public:
    MmapedV4l2Buffer(const std::byte *pData, uint32_t dataSize, uint32_t width, uint32_t height,
                     uint32_t bytesPerLine, uint32_t pixFormat);

    virtual ~MmapedV4l2Buffer();

    [[maybe_unused]] [[nodiscard]] const std::byte *getPData() const {
      return p_data_;
    }

    [[maybe_unused]] [[nodiscard]] uint32_t getDataSize() const {
      return data_size_;
    }

    [[nodiscard]] uint32_t getWidth() const {
      return width_;
    }

    [[nodiscard]] uint32_t getHeight() const {
      return height_;
    }

    [[maybe_unused]] [[nodiscard]] uint32_t getBytesPerLine() const {
      return bytes_per_line_;
    }

    [[maybe_unused]] [[nodiscard]] uint32_t getPixFormat() const {
      return pix_format_;
    }
  };

  struct img_params_t {
    uint32_t img_size;
    uint32_t width;
    uint32_t height;
    uint32_t bytes_per_line;
    uint32_t pix_format;
  };

  struct CtrlParameter {
    const std::string name;
    const uint32_t v4l2_id;

    const uint32_t minimum_val;
    const uint32_t maximum_val;
    const uint32_t default_val;
    CtrlParameter(std::string name,
                  const uint32_t v_4_l_2_id,
                  const uint32_t minimum_val,
                  const uint32_t maximum_val,
                  const uint32_t default_val);

    inline CtrlParameter(const CtrlParameter &) = default;
    inline CtrlParameter &operator=(const CtrlParameter &o) {
      const_cast<std::string &>(name) = o.name;
      const_cast<uint32_t &>(v4l2_id) = o.v4l2_id;
      const_cast<uint32_t &>(minimum_val) = o.minimum_val;
      const_cast<uint32_t &>(maximum_val) = o.maximum_val;
      const_cast<uint32_t &>(default_val) = o.default_val;

      return *this;
    }
  };

  struct mode_t {
    mode_t(const uint32_t pix_format,
           const uint_fast16_t width,
           const uint_fast16_t height,
           const uint32_t camera_mode_index,
           const float frame_rate,
           bool compressed,
           bool emulated);
    const uint32_t pix_format;
    const uint_fast16_t width;
    const uint_fast16_t height;
    const uint32_t camera_mode_index;
    const float frame_rate;
    bool compressed;
    bool emulated;
  };

  struct V4l2CtlVal {
    const uint32_t current_val;
    const uint32_t minimum_val;
    const uint32_t maximum_val;
    const uint32_t default_val;

    V4l2CtlVal(uint32_t current_val,
               uint32_t minimum_val,
               uint32_t maximum_val,
               uint32_t default_val);
  };

 private:
  std::vector<MmapedV4l2Buffer> frames_;
  std::vector<V4l2Camera::CtrlParameter> ctrl_parameters_;
  const std::string dev_path_;
  int fd_;

 private:
  void openDevice();
  void closeDevice() const;

  void checkCapabilities() const;

  void requestBufferAllocationFromDriver(uint buffers_count);
  void mmapFramesBufferAllocatedByDriver(uint buffers_count, const img_params_t &img_params);
  void initFrames(uint buffers_count);
  void cleanFrames();

  void pushFramesToDriverQueue();
  void enableCapturing() const;
  void disableCapturing() const;

  void updateCtrlParametersVector();

  template<char const ctl_name[]>
  V4l2Camera::V4l2CtlVal setV4l2Ctrl(int value);

  template<char const ctl_name[]>
  V4l2Camera::V4l2CtlVal getV4l2Ctrl();


//  void ();

 public:

#ifdef WITH_PRINTING // REQ BOOST FORMAT
  [[maybe_unused]] void printDevCapabilities() const;

  [[maybe_unused]] void printSupportedImageFormats() const;

  [[maybe_unused]] void printCropCapabilities() const;

  [[maybe_unused]] void printCtrlsList() const;
#endif

  explicit V4l2Camera(std::string devPath);

  virtual ~V4l2Camera();

  void open();

  void startCapturing(uint buffers_count = DEFAULT_BUFFERS_COUNT);

  void stopCapturing();

  void close();

  [[nodiscard]] img_params_t getImageParams() const;

  [[maybe_unused]] [[nodiscard]] const std::string &getDevPath() const;

  [[maybe_unused]] [[nodiscard]] std::vector<V4l2Camera::mode_t> getSensorModes() const;

  [[maybe_unused]] void setSensorMode(const mode_t &m);

  const V4l2Camera::MmapedV4l2Buffer &getNewImage();

  [[maybe_unused]] const std::vector<V4l2Camera::CtrlParameter> &getControlParameters();

  using Gain = V4l2CtlVal;
  [[maybe_unused]] Gain getGain();
  [[maybe_unused]] Gain setGain(uint32_t gain);

  using Exposure = V4l2CtlVal;
  [[maybe_unused]] Exposure getExposure();
  [[maybe_unused]] Exposure setExposure(uint32_t exposure);


//  [[maybe_unused]] getControlParameters();
};

#endif //V4L2_EXAMPLE_V4L2CAMERA_HPP
