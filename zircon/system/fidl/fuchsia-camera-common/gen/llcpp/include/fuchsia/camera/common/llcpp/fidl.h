// WARNING: This file is machine generated by fidlgen.

#pragma once

#include <lib/fidl/internal.h>
#include <lib/fidl/cpp/vector_view.h>
#include <lib/fidl/cpp/string_view.h>
#include <lib/fidl/llcpp/array.h>
#include <lib/fidl/llcpp/coding.h>
#include <lib/fidl/llcpp/traits.h>
#include <lib/fidl/llcpp/transaction.h>
#include <lib/fit/function.h>
#include <lib/zx/channel.h>
#include <zircon/fidl.h>

#include <fuchsia/sysmem/llcpp/fidl.h>

namespace llcpp {

namespace fuchsia {
namespace camera {
namespace common {

struct RealWorldStreamConfig;
struct Metadata;
enum class FrameStatus : uint32_t {
  OK = 1u,
  ERROR_FRAME = 2u,
  ERROR_BUFFER_FULL = 3u,
};


struct FrameAvailableEvent;
class Stream;
struct FrameRate;
struct VideoFormat;
struct DeviceInfo;
struct ArtificialStreamConfig;
struct VirtualStreamConfig;
struct VirtualCameraConfig;
class VirtualCameraFactory;



// Configuration for a stream generated from stored frames.
// TODO(eweeks): Replace this stand-in with the full design.
struct RealWorldStreamConfig {
  static constexpr const fidl_type_t* Type = nullptr;
  static constexpr uint32_t MaxNumHandles = 0;
  static constexpr uint32_t PrimarySize = 2;
  [[maybe_unused]]
  static constexpr uint32_t MaxOutOfLine = 0;

  // Numeric identifier for the stream being configured.
  int16_t stream_id{};
};



// Extra information associated with the frame.
struct Metadata {
  static constexpr const fidl_type_t* Type = nullptr;
  static constexpr uint32_t MaxNumHandles = 0;
  static constexpr uint32_t PrimarySize = 8;
  [[maybe_unused]]
  static constexpr uint32_t MaxOutOfLine = 0;

  int64_t timestamp{};
};



// Sent by the driver to the client when a frame is available for processing,
// or an error occurred.
struct FrameAvailableEvent {
  static constexpr const fidl_type_t* Type = nullptr;
  static constexpr uint32_t MaxNumHandles = 0;
  static constexpr uint32_t PrimarySize = 16;
  [[maybe_unused]]
  static constexpr uint32_t MaxOutOfLine = 0;

  // Non zero if an error occurred.
  FrameStatus frame_status{};

  // The index of the buffer in the buffer collection.
  uint32_t buffer_id{};

  // Any associated metadata such as timestamp.
  Metadata metadata{};
};

extern "C" const fidl_type_t fuchsia_camera_common_StreamReleaseFrameRequestTable;

// Protocol shared between the driver and the consumer.
class Stream final {
 public:

  using StartRequest = ::fidl::AnyZeroArgMessage;

  using StopRequest = ::fidl::AnyZeroArgMessage;

  struct ReleaseFrameRequest final {
    FIDL_ALIGNDECL
    fidl_message_header_t _hdr;
    uint32_t buffer_id;

    static constexpr const fidl_type_t* Type = &fuchsia_camera_common_StreamReleaseFrameRequestTable;
    static constexpr uint32_t MaxNumHandles = 0;
    static constexpr uint32_t PrimarySize = 24;
    static constexpr uint32_t MaxOutOfLine = 0;
  };

  struct OnFrameAvailableResponse final {
    FIDL_ALIGNDECL
    fidl_message_header_t _hdr;
    FrameAvailableEvent frame;

    static constexpr const fidl_type_t* Type = nullptr;
    static constexpr uint32_t MaxNumHandles = 0;
    static constexpr uint32_t PrimarySize = 32;
    static constexpr uint32_t MaxOutOfLine = 0;
  };

  struct EventHandlers {
    // Sent by the driver to the client when a frame is available for processing,
    // or an error occurred.
    fit::function<zx_status_t(FrameAvailableEvent frame)> on_frame_available;

    // Fallback handler when an unknown ordinal is received.
    // Caller may put custom error handling logic here.
    fit::function<zx_status_t()> unknown;
  };

  class SyncClient final {
   public:
    SyncClient(::zx::channel channel) : channel_(std::move(channel)) {}

    ~SyncClient() {}

    // Starts the streaming of frames.
    zx_status_t Start();

    // Stops the streaming of frames.
    zx_status_t Stop();

    // Unlocks the specified frame, allowing the driver to reuse the memory.
    zx_status_t ReleaseFrame(uint32_t buffer_id);

    // Unlocks the specified frame, allowing the driver to reuse the memory.
    // Caller provides the backing storage for FIDL message via request and response buffers.
    zx_status_t ReleaseFrame(::fidl::BytePart _request_buffer, uint32_t buffer_id);

    // Unlocks the specified frame, allowing the driver to reuse the memory.
    // Messages are encoded and decoded in-place.
    zx_status_t ReleaseFrame(::fidl::DecodedMessage<ReleaseFrameRequest> params);

    // Handle all possible events defined in this protocol.
    // Blocks to consume exactly one message from the channel, then call the corresponding handler
    // defined in |EventHandlers|. The return status of the handler function is folded with any
    // transport-level errors and returned.
    zx_status_t HandleEvents(EventHandlers handlers);
   private:
    ::zx::channel channel_;
  };

  // Methods to make a sync FIDL call directly on an unowned channel, avoiding setting up a client.
  class Call final {
   public:

    // Starts the streaming of frames.
    static zx_status_t Start(zx::unowned_channel _client_end);

    // Stops the streaming of frames.
    static zx_status_t Stop(zx::unowned_channel _client_end);

    // Unlocks the specified frame, allowing the driver to reuse the memory.
    static zx_status_t ReleaseFrame(zx::unowned_channel _client_end, uint32_t buffer_id);

    // Unlocks the specified frame, allowing the driver to reuse the memory.
    // Caller provides the backing storage for FIDL message via request and response buffers.
    static zx_status_t ReleaseFrame(zx::unowned_channel _client_end, ::fidl::BytePart _request_buffer, uint32_t buffer_id);

    // Unlocks the specified frame, allowing the driver to reuse the memory.
    // Messages are encoded and decoded in-place.
    static zx_status_t ReleaseFrame(zx::unowned_channel _client_end, ::fidl::DecodedMessage<ReleaseFrameRequest> params);

    // Handle all possible events defined in this protocol.
    // Blocks to consume exactly one message from the channel, then call the corresponding handler
    // defined in |EventHandlers|. The return status of the handler function is folded with any
    // transport-level errors and returned.
    static zx_status_t HandleEvents(zx::unowned_channel client_end, EventHandlers handlers);
  };

  // Pure-virtual interface to be implemented by a server.
  class Interface {
   public:
    Interface() = default;
    virtual ~Interface() = default;
    using _Outer = Stream;
    using _Base = ::fidl::CompleterBase;

    using StartCompleter = ::fidl::Completer<>;

    virtual void Start(StartCompleter::Sync _completer) = 0;

    using StopCompleter = ::fidl::Completer<>;

    virtual void Stop(StopCompleter::Sync _completer) = 0;

    using ReleaseFrameCompleter = ::fidl::Completer<>;

    virtual void ReleaseFrame(uint32_t buffer_id, ReleaseFrameCompleter::Sync _completer) = 0;

  };

  // Attempts to dispatch the incoming message to a handler function in the server implementation.
  // If there is no matching handler, it returns false, leaving the message and transaction intact.
  // In all other cases, it consumes the message and returns true.
  // It is possible to chain multiple TryDispatch functions in this manner.
  static bool TryDispatch(Interface* impl, fidl_msg_t* msg, ::fidl::Transaction* txn);

  // Dispatches the incoming message to one of the handlers functions in the interface.
  // If there is no matching handler, it closes all the handles in |msg| and closes the channel with
  // a |ZX_ERR_NOT_SUPPORTED| epitaph, before returning false. The message should then be discarded.
  static bool Dispatch(Interface* impl, fidl_msg_t* msg, ::fidl::Transaction* txn);

  // Same as |Dispatch|, but takes a |void*| instead of |Interface*|. Only used with |fidl::Bind|
  // to reduce template expansion.
  // Do not call this method manually. Use |Dispatch| instead.
  static bool TypeErasedDispatch(void* impl, fidl_msg_t* msg, ::fidl::Transaction* txn) {
    return Dispatch(static_cast<Interface*>(impl), msg, txn);
  }

  // Sent by the driver to the client when a frame is available for processing,
  // or an error occurred.
  static zx_status_t SendOnFrameAvailableEvent(::zx::unowned_channel _chan, FrameAvailableEvent frame);

  // Sent by the driver to the client when a frame is available for processing,
  // or an error occurred.
  // Caller provides the backing storage for FIDL message via response buffers.
  static zx_status_t SendOnFrameAvailableEvent(::zx::unowned_channel _chan, ::fidl::BytePart _buffer, FrameAvailableEvent frame);

  // Sent by the driver to the client when a frame is available for processing,
  // or an error occurred.
  // Messages are encoded in-place.
  static zx_status_t SendOnFrameAvailableEvent(::zx::unowned_channel _chan, ::fidl::DecodedMessage<OnFrameAvailableResponse> params);

};



// The number of frames being produced every second.
struct FrameRate {
  static constexpr const fidl_type_t* Type = nullptr;
  static constexpr uint32_t MaxNumHandles = 0;
  static constexpr uint32_t PrimarySize = 8;
  [[maybe_unused]]
  static constexpr uint32_t MaxOutOfLine = 0;

  // The frame rate is frames_per_sec_numerator / frames_per_sec_denominator.
  uint32_t frames_per_sec_numerator{};

  uint32_t frames_per_sec_denominator{};
};

extern "C" const fidl_type_t fuchsia_camera_common_VideoFormatTable;

// Video format includes the image format and frame rate of frames being produced.
struct VideoFormat {
  static constexpr const fidl_type_t* Type = &fuchsia_camera_common_VideoFormatTable;
  static constexpr uint32_t MaxNumHandles = 0;
  static constexpr uint32_t PrimarySize = 80;
  [[maybe_unused]]
  static constexpr uint32_t MaxOutOfLine = 0;

  ::llcpp::fuchsia::sysmem::ImageFormat format{};

  FrameRate rate{};
};

extern "C" const fidl_type_t fuchsia_camera_common_DeviceInfoTable;

// Identifying information about the device.
struct DeviceInfo {
  static constexpr const fidl_type_t* Type = &fuchsia_camera_common_DeviceInfoTable;
  static constexpr uint32_t MaxNumHandles = 0;
  static constexpr uint32_t PrimarySize = 24;
  [[maybe_unused]]
  static constexpr uint32_t MaxOutOfLine = 0;

  // Currently populated by the camera manager
  uint64_t camera_id{};

  uint16_t vendor_id{};

  // string vendor_name;
  uint16_t product_id{};

  // string product_name;
  // string serial_number;
  // The maximum number of stream interfaces that the device can support
  // simultaneously.
  uint16_t max_stream_count{};

  uint32_t output_capabilities{};
};

constexpr uint32_t CAMERA_OUTPUT_UNKNOWN = 0u;

constexpr uint32_t CAMERA_OUTPUT_STREAM = 4u;

constexpr uint32_t CAMERA_OUTPUT_STILL_IMAGE = 1u;

constexpr uint32_t CAMERA_OUTPUT_STEREO = 32u;

constexpr uint32_t CAMERA_OUTPUT_HDR = 8u;

constexpr uint32_t CAMERA_OUTPUT_DEPTH = 16u;

constexpr uint32_t CAMERA_OUTPUT_BURST = 2u;



// Configuration for an artificial stream.
// TODO(eweeks): Replace this stand-in with the full design.
struct ArtificialStreamConfig {
  static constexpr const fidl_type_t* Type = nullptr;
  static constexpr uint32_t MaxNumHandles = 0;
  static constexpr uint32_t PrimarySize = 2;
  [[maybe_unused]]
  static constexpr uint32_t MaxOutOfLine = 0;

  // Numeric identifier for the stream being configured.
  int16_t stream_id{};
};

extern "C" const fidl_type_t fuchsia_camera_common_VirtualStreamConfigTable;

// Configuration for the stream.
// The Configuration must be either artificial or real
// TODO(eweeks): Replace this stand-in with the full design.
struct VirtualStreamConfig {
  VirtualStreamConfig() : ordinal_(Tag::kUnknown), envelope_{} {}

  enum class Tag : fidl_xunion_tag_t {
    kUnknown = 0,
    kArtificialConfig = 1033488457,  // 0x3d99c849
    kRealWorldConfig = 308976604,  // 0x126a9bdc
  };

  bool is_artificial_config() const { return ordinal_ == Tag::kArtificialConfig; }

  void set_artificial_config(ArtificialStreamConfig* elem) {
    ordinal_ = Tag::kArtificialConfig;
    envelope_.data = static_cast<void*>(elem);
  }

  ArtificialStreamConfig& artificial_config() const {
    ZX_ASSERT(ordinal_ == Tag::kArtificialConfig);
    return *static_cast<ArtificialStreamConfig*>(envelope_.data);
  }

  bool is_real_world_config() const { return ordinal_ == Tag::kRealWorldConfig; }

  void set_real_world_config(RealWorldStreamConfig* elem) {
    ordinal_ = Tag::kRealWorldConfig;
    envelope_.data = static_cast<void*>(elem);
  }

  RealWorldStreamConfig& real_world_config() const {
    ZX_ASSERT(ordinal_ == Tag::kRealWorldConfig);
    return *static_cast<RealWorldStreamConfig*>(envelope_.data);
  }

  Tag which() const;

  static constexpr const fidl_type_t* Type = &fuchsia_camera_common_VirtualStreamConfigTable;
  static constexpr uint32_t MaxNumHandles = 0;
  static constexpr uint32_t PrimarySize = 24;
  [[maybe_unused]]
  static constexpr uint32_t MaxOutOfLine = 8;

 private:
  static void SizeAndOffsetAssertionHelper();
  Tag ordinal_;
  FIDL_ALIGNDECL
  fidl_envelope_t envelope_;
};

extern "C" const fidl_type_t fuchsia_camera_common_VirtualCameraConfigTable;

// Configuration used by VirtualManager to create a VirtualCameraDevice.
struct VirtualCameraConfig {
  static constexpr const fidl_type_t* Type = &fuchsia_camera_common_VirtualCameraConfigTable;
  static constexpr uint32_t MaxNumHandles = 0;
  static constexpr uint32_t PrimarySize = 64;
  [[maybe_unused]]
  static constexpr uint32_t MaxOutOfLine = 4294967295;

  // The formats that the controller will support.
  ::fidl::VectorView<VideoFormat> formats{};

  // Device specific information that can be set by the user.
  DeviceInfo info{};

  // Either an ArtificialStreamConfig or a RealWorldStreamConfig.
  VirtualStreamConfig stream_config{};
};

extern "C" const fidl_type_t fuchsia_camera_common_VirtualCameraFactoryCreateDeviceRequestTable;

// Protocol for managing virtual cameras that need to be added for tests.
class VirtualCameraFactory final {
 public:

  struct CreateDeviceRequest final {
    FIDL_ALIGNDECL
    fidl_message_header_t _hdr;
    VirtualCameraConfig config;

    static constexpr const fidl_type_t* Type = &fuchsia_camera_common_VirtualCameraFactoryCreateDeviceRequestTable;
    static constexpr uint32_t MaxNumHandles = 0;
    static constexpr uint32_t PrimarySize = 80;
    static constexpr uint32_t MaxOutOfLine = 4294967295;
  };


  class SyncClient final {
   public:
    SyncClient(::zx::channel channel) : channel_(std::move(channel)) {}

    ~SyncClient() {}

    // Creates a new VirtualCameraDevice based on the configuration passed in.
    // `config`: a VirtualCameraConfig defining how the new device should behave.
    zx_status_t CreateDevice(VirtualCameraConfig config);

    // Creates a new VirtualCameraDevice based on the configuration passed in.
    // `config`: a VirtualCameraConfig defining how the new device should behave.
    // Caller provides the backing storage for FIDL message via request and response buffers.
    zx_status_t CreateDevice(::fidl::BytePart _request_buffer, VirtualCameraConfig config);

    // Creates a new VirtualCameraDevice based on the configuration passed in.
    // `config`: a VirtualCameraConfig defining how the new device should behave.
    // Messages are encoded and decoded in-place.
    zx_status_t CreateDevice(::fidl::DecodedMessage<CreateDeviceRequest> params);

   private:
    ::zx::channel channel_;
  };

  // Methods to make a sync FIDL call directly on an unowned channel, avoiding setting up a client.
  class Call final {
   public:

    // Creates a new VirtualCameraDevice based on the configuration passed in.
    // `config`: a VirtualCameraConfig defining how the new device should behave.
    static zx_status_t CreateDevice(zx::unowned_channel _client_end, VirtualCameraConfig config);

    // Creates a new VirtualCameraDevice based on the configuration passed in.
    // `config`: a VirtualCameraConfig defining how the new device should behave.
    // Caller provides the backing storage for FIDL message via request and response buffers.
    static zx_status_t CreateDevice(zx::unowned_channel _client_end, ::fidl::BytePart _request_buffer, VirtualCameraConfig config);

    // Creates a new VirtualCameraDevice based on the configuration passed in.
    // `config`: a VirtualCameraConfig defining how the new device should behave.
    // Messages are encoded and decoded in-place.
    static zx_status_t CreateDevice(zx::unowned_channel _client_end, ::fidl::DecodedMessage<CreateDeviceRequest> params);

  };

  // Pure-virtual interface to be implemented by a server.
  class Interface {
   public:
    Interface() = default;
    virtual ~Interface() = default;
    using _Outer = VirtualCameraFactory;
    using _Base = ::fidl::CompleterBase;

    using CreateDeviceCompleter = ::fidl::Completer<>;

    virtual void CreateDevice(VirtualCameraConfig config, CreateDeviceCompleter::Sync _completer) = 0;

  };

  // Attempts to dispatch the incoming message to a handler function in the server implementation.
  // If there is no matching handler, it returns false, leaving the message and transaction intact.
  // In all other cases, it consumes the message and returns true.
  // It is possible to chain multiple TryDispatch functions in this manner.
  static bool TryDispatch(Interface* impl, fidl_msg_t* msg, ::fidl::Transaction* txn);

  // Dispatches the incoming message to one of the handlers functions in the interface.
  // If there is no matching handler, it closes all the handles in |msg| and closes the channel with
  // a |ZX_ERR_NOT_SUPPORTED| epitaph, before returning false. The message should then be discarded.
  static bool Dispatch(Interface* impl, fidl_msg_t* msg, ::fidl::Transaction* txn);

  // Same as |Dispatch|, but takes a |void*| instead of |Interface*|. Only used with |fidl::Bind|
  // to reduce template expansion.
  // Do not call this method manually. Use |Dispatch| instead.
  static bool TypeErasedDispatch(void* impl, fidl_msg_t* msg, ::fidl::Transaction* txn) {
    return Dispatch(static_cast<Interface*>(impl), msg, txn);
  }

};

}  // namespace common
}  // namespace camera
}  // namespace fuchsia
}  // namespace llcpp

namespace fidl {

template <>
struct IsFidlType<::llcpp::fuchsia::camera::common::RealWorldStreamConfig> : public std::true_type {};
static_assert(std::is_standard_layout_v<::llcpp::fuchsia::camera::common::RealWorldStreamConfig>);
static_assert(offsetof(::llcpp::fuchsia::camera::common::RealWorldStreamConfig, stream_id) == 0);
static_assert(sizeof(::llcpp::fuchsia::camera::common::RealWorldStreamConfig) == ::llcpp::fuchsia::camera::common::RealWorldStreamConfig::PrimarySize);

template <>
struct IsFidlType<::llcpp::fuchsia::camera::common::Metadata> : public std::true_type {};
static_assert(std::is_standard_layout_v<::llcpp::fuchsia::camera::common::Metadata>);
static_assert(offsetof(::llcpp::fuchsia::camera::common::Metadata, timestamp) == 0);
static_assert(sizeof(::llcpp::fuchsia::camera::common::Metadata) == ::llcpp::fuchsia::camera::common::Metadata::PrimarySize);

template <>
struct IsFidlType<::llcpp::fuchsia::camera::common::FrameAvailableEvent> : public std::true_type {};
static_assert(std::is_standard_layout_v<::llcpp::fuchsia::camera::common::FrameAvailableEvent>);
static_assert(offsetof(::llcpp::fuchsia::camera::common::FrameAvailableEvent, frame_status) == 0);
static_assert(offsetof(::llcpp::fuchsia::camera::common::FrameAvailableEvent, buffer_id) == 4);
static_assert(offsetof(::llcpp::fuchsia::camera::common::FrameAvailableEvent, metadata) == 8);
static_assert(sizeof(::llcpp::fuchsia::camera::common::FrameAvailableEvent) == ::llcpp::fuchsia::camera::common::FrameAvailableEvent::PrimarySize);

template <>
struct IsFidlType<::llcpp::fuchsia::camera::common::Stream::ReleaseFrameRequest> : public std::true_type {};
template <>
struct IsFidlMessage<::llcpp::fuchsia::camera::common::Stream::ReleaseFrameRequest> : public std::true_type {};
static_assert(sizeof(::llcpp::fuchsia::camera::common::Stream::ReleaseFrameRequest)
    == ::llcpp::fuchsia::camera::common::Stream::ReleaseFrameRequest::PrimarySize);
static_assert(offsetof(::llcpp::fuchsia::camera::common::Stream::ReleaseFrameRequest, buffer_id) == 16);

template <>
struct IsFidlType<::llcpp::fuchsia::camera::common::Stream::OnFrameAvailableResponse> : public std::true_type {};
template <>
struct IsFidlMessage<::llcpp::fuchsia::camera::common::Stream::OnFrameAvailableResponse> : public std::true_type {};
static_assert(sizeof(::llcpp::fuchsia::camera::common::Stream::OnFrameAvailableResponse)
    == ::llcpp::fuchsia::camera::common::Stream::OnFrameAvailableResponse::PrimarySize);
static_assert(offsetof(::llcpp::fuchsia::camera::common::Stream::OnFrameAvailableResponse, frame) == 16);

template <>
struct IsFidlType<::llcpp::fuchsia::camera::common::FrameRate> : public std::true_type {};
static_assert(std::is_standard_layout_v<::llcpp::fuchsia::camera::common::FrameRate>);
static_assert(offsetof(::llcpp::fuchsia::camera::common::FrameRate, frames_per_sec_numerator) == 0);
static_assert(offsetof(::llcpp::fuchsia::camera::common::FrameRate, frames_per_sec_denominator) == 4);
static_assert(sizeof(::llcpp::fuchsia::camera::common::FrameRate) == ::llcpp::fuchsia::camera::common::FrameRate::PrimarySize);

template <>
struct IsFidlType<::llcpp::fuchsia::camera::common::VideoFormat> : public std::true_type {};
static_assert(std::is_standard_layout_v<::llcpp::fuchsia::camera::common::VideoFormat>);
static_assert(offsetof(::llcpp::fuchsia::camera::common::VideoFormat, format) == 0);
static_assert(offsetof(::llcpp::fuchsia::camera::common::VideoFormat, rate) == 72);
static_assert(sizeof(::llcpp::fuchsia::camera::common::VideoFormat) == ::llcpp::fuchsia::camera::common::VideoFormat::PrimarySize);

template <>
struct IsFidlType<::llcpp::fuchsia::camera::common::DeviceInfo> : public std::true_type {};
static_assert(std::is_standard_layout_v<::llcpp::fuchsia::camera::common::DeviceInfo>);
static_assert(offsetof(::llcpp::fuchsia::camera::common::DeviceInfo, camera_id) == 0);
static_assert(offsetof(::llcpp::fuchsia::camera::common::DeviceInfo, vendor_id) == 8);
static_assert(offsetof(::llcpp::fuchsia::camera::common::DeviceInfo, product_id) == 10);
static_assert(offsetof(::llcpp::fuchsia::camera::common::DeviceInfo, max_stream_count) == 12);
static_assert(offsetof(::llcpp::fuchsia::camera::common::DeviceInfo, output_capabilities) == 16);
static_assert(sizeof(::llcpp::fuchsia::camera::common::DeviceInfo) == ::llcpp::fuchsia::camera::common::DeviceInfo::PrimarySize);

template <>
struct IsFidlType<::llcpp::fuchsia::camera::common::ArtificialStreamConfig> : public std::true_type {};
static_assert(std::is_standard_layout_v<::llcpp::fuchsia::camera::common::ArtificialStreamConfig>);
static_assert(offsetof(::llcpp::fuchsia::camera::common::ArtificialStreamConfig, stream_id) == 0);
static_assert(sizeof(::llcpp::fuchsia::camera::common::ArtificialStreamConfig) == ::llcpp::fuchsia::camera::common::ArtificialStreamConfig::PrimarySize);

template <>
struct IsFidlType<::llcpp::fuchsia::camera::common::VirtualStreamConfig> : public std::true_type {};
static_assert(std::is_standard_layout_v<::llcpp::fuchsia::camera::common::VirtualStreamConfig>);

template <>
struct IsFidlType<::llcpp::fuchsia::camera::common::VirtualCameraConfig> : public std::true_type {};
static_assert(std::is_standard_layout_v<::llcpp::fuchsia::camera::common::VirtualCameraConfig>);
static_assert(offsetof(::llcpp::fuchsia::camera::common::VirtualCameraConfig, formats) == 0);
static_assert(offsetof(::llcpp::fuchsia::camera::common::VirtualCameraConfig, info) == 16);
static_assert(offsetof(::llcpp::fuchsia::camera::common::VirtualCameraConfig, stream_config) == 40);
static_assert(sizeof(::llcpp::fuchsia::camera::common::VirtualCameraConfig) == ::llcpp::fuchsia::camera::common::VirtualCameraConfig::PrimarySize);

template <>
struct IsFidlType<::llcpp::fuchsia::camera::common::VirtualCameraFactory::CreateDeviceRequest> : public std::true_type {};
template <>
struct IsFidlMessage<::llcpp::fuchsia::camera::common::VirtualCameraFactory::CreateDeviceRequest> : public std::true_type {};
static_assert(sizeof(::llcpp::fuchsia::camera::common::VirtualCameraFactory::CreateDeviceRequest)
    == ::llcpp::fuchsia::camera::common::VirtualCameraFactory::CreateDeviceRequest::PrimarySize);
static_assert(offsetof(::llcpp::fuchsia::camera::common::VirtualCameraFactory::CreateDeviceRequest, config) == 16);

}  // namespace fidl
