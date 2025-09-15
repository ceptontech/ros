/*
  Copyright Cepton Technologies Inc., All rights reserved.

  Cepton Sensor SDK C interface.
*/
#ifndef CEPTON_SDK3_H
#define CEPTON_SDK3_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// API compatibility version (Not the same as SDK version)
#define CEPTON_API_VERSION 205

//------------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------------

// CEPTON_EXPORT
#ifndef CEPTON_EXPORT
#ifdef _MSC_VER
#define CEPTON_EXPORT __declspec(dllimport)
#else
#define CEPTON_EXPORT
#endif
#endif

//------------------------------------------------------------------------------
// Errors
//------------------------------------------------------------------------------

// Error code returned by most library functions.
#ifndef CEPTON_SDK_H
enum {
  // No error.
  CEPTON_SUCCESS = 0,
  // Generic error.
  CEPTON_ERROR_GENERIC = -1,
  // Failed to allocate heap memory.
  CEPTON_ERROR_OUT_OF_MEMORY = -2,
  // Could not find sensor.
  CEPTON_ERROR_SENSOR_NOT_FOUND = -4,
  // SDK version mismatch.
  CEPTON_ERROR_SDK_VERSION_MISMATCH = -5,
  // Networking error.
  CEPTON_ERROR_COMMUNICATION = -6,
  // Callback already set.
  CEPTON_ERROR_TOO_MANY_CALLBACKS = -7,
  // Invalid value or uninitialized struct.
  CEPTON_ERROR_INVALID_ARGUMENTS = -8,
  // Already initialized.
  CEPTON_ERROR_ALREADY_INITIALIZED = -9,
  // Not initialized.
  CEPTON_ERROR_NOT_INITIALIZED = -10,
  // Invalid file type.
  CEPTON_ERROR_INVALID_FILE_TYPE = -11,
  // File IO error.
  CEPTON_ERROR_FILE_IO = -12,
  // Corrupt/invalid file.
  CEPTON_ERROR_CORRUPT_FILE = -13,
  // Not open.
  CEPTON_ERROR_NOT_OPEN = -14,
  // End of file.
  CEPTON_ERROR_EOF = -15,
  // Functionality not supported by device
  CEPTON_ERROR_NOT_SUPPORTED = -16,
  // Device response invalid
  CEPTON_ERROR_INVALID_RESPONSE = -17,
  // Software state invalid
  CEPTON_ERROR_INVALID_STATE = -18,
  // Feature is not enabled
  CEPTON_ERROR_NOT_ENABLED = -19,
  // Get data timeout
  CEPTON_ERROR_TIMEOUT = -20,
};
#endif  // CEPTON_SDK_H

//------------------------------------------------------------------------------
// Static functions that can be called before initialization
//------------------------------------------------------------------------------

/**
 * Returns empty string if error code is invalid.
 *
 * @return Error code name string converted from int. "" if unrecognized.
 */
CEPTON_EXPORT const char *CeptonGetErrorCodeName(int error_code);

/**
 * Get the SDK version (not the same as API version)
 * @return major/minor/patch/build in 4 bytes (little endian 32bit integer).
           e.g. 2.1.15.0 will be 0x000F0102.
 */
CEPTON_EXPORT uint32_t CeptonGetSdkVersion(void);

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------

/**
 * Sensor identifier. Generated from sensor IP address.
 */
typedef uint64_t CeptonSensorHandle;

// Sensor model.
typedef uint16_t CeptonSensorModel;
enum {
  CEPTON_MODEL_VISTA_860_GEN2 = 7,
  CEPTON_MODEL_VISTA_X120 = 10,
  CEPTON_MODEL_SORA_P60 = 11,
  CEPTON_MODEL_VISTA_P60 = 12,
  // 13 is reserved
  CEPTON_MODEL_VISTA_P90 = 14,
  CEPTON_MODEL_SORA_P90 = 15,
  CEPTON_MODEL_VISTA_P61 = 16,
  CEPTON_MODEL_SORA_P61 = 17,
  CEPTON_MODEL_NOVA_A = 18,
  // 19 is reserved for VISTA_P60 Rev2 firmware releases
  // 20 is reserved
  // 21 is reserved
  CEPTON_MODEL_APEX90 = 22,
  // 23 is reserved
  CEPTON_MODEL_VISTA_X90 = 24,
  CEPTON_MODEL_NOVA_B1 = 25,
  CEPTON_MODEL_TERA_TALL = 26,
  CEPTON_MODEL_TERA_SHORT = 27,
  CEPTON_MODEL_SORA_X90 = 28,
  // 29 is reserved
  CEPTON_MODEL_NOVA_B2 = 30,
  CEPTON_MODEL_TERA_A2 = 31,
  CEPTON_MODEL_VISTA_X90_PLUS = 32,
  CEPTON_MODEL_ULTRA = 33,
  CEPTON_MODEL_ULTRA_B2 = 34,
  CEPTON_MODEL_NOVA_B3 = 35,
  CEPTON_MODEL_VISTA_ULTRA = 36,
  CEPTON_MODEL_NOVA_ULTRA = 37,
};

enum {
  CEPTON_POINT_SATURATED = 1 << 0,
  CEPTON_POINT_BLOOMING = 1 << 1,
  CEPTON_POINT_FRAME_PARITY = 1 << 2,
  CEPTON_POINT_FRAME_BOUNDARY = 1 << 3,
  CEPTON_POINT_SECOND_RETURN = 1 << 4,
  CEPTON_POINT_NO_RETURN = 1 << 5,
  CEPTON_POINT_NOISE = 1 << 6,
  CEPTON_POINT_BLOCKED = 1 << 7,
};

#pragma pack(push, 1)
// struct CeptonPoint {
//   int16_t x;
//   uint16_t y;
//   int16_t z;
//   uint8_t reflectivity;
//   uint8_t relative_timestamp;
//   uint8_t channel_id;
//   uint8_t flags;
// };

struct CeptonPointEx {
  int32_t x;  // Unit is 1/65536 m or ~0.015mm
  int32_t y;
  int32_t z;
  uint16_t reflectivity;        // Unit is 1%, no lookup table
  uint16_t relative_timestamp;  // Unit is 1 us
  uint16_t flags;
  uint16_t channel_id;
};

struct CeptonSensor {
  // Size of the CeptonSensor struct
  // plus any consecutive sensor info blocks
  uint32_t info_size;

  // per sensor info
  uint32_t serial_number;
  CeptonSensorHandle handle;

  // Model
  unsigned char model_name[28];
  uint16_t model;
  uint16_t model_reserved;
  uint32_t part_number;

  // FW
  uint32_t firmware_version;  // LSB->MSB major/minor/build/patch

  // Time
  int64_t power_up_timestamp;
  int64_t time_sync_offset;
  int64_t time_sync_drift;

  // Config
  uint8_t return_count;
  uint8_t channel_count;
  uint16_t reserved;
  uint32_t status_flags;

  // Unit in 0.01 Kelvin
  uint16_t temperature;

  // Fault summary about sensor status.
  // Can be interpreted using CEPTON_FAULT_SUMMARY_*** bitmasks
  uint32_t fault_summary;
  // Additional Detail about each fault the sensor can report
  uint8_t fault_entries[32];
};

struct CeptonPanicMessage {
  CeptonSensorHandle handle;
  uint32_t serial_number;
  uint32_t fault_identity;
  uint32_t life_counter;
  uint64_t ptp_timestamp;
};

#pragma pack(pop)

const uint32_t CEPTON_FAULT_SUMMARY_DATA_RATIONALITY = 1 << 0;
const uint32_t CEPTON_FAULT_SUMMARY_DATA_CHECKSUM = 1 << 1;
const uint32_t CEPTON_FAULT_SUMMARY_TEMPERATURE_RANGE = 1 << 2;
const uint32_t CEPTON_FAULT_SUMMARY_VOLTAGE_RANGE = 1 << 3;

enum _CeptonSensorStatusFlags {
  CEPTON_SENSOR_PTP_CONNECTED = 1,
  CEPTON_SENSOR_PPS_CONNECTED = 2,
  CEPTON_SENSOR_NMEA_CONNECTED = 4,
};

//------------------------------------------------------------------------------
// Initialization
//------------------------------------------------------------------------------

/**
 * Callback for receiving SDK and sensor errors.
 * This is a global function (without user_data).
 * @param handle sensor handle.
 * @param error_code Error code.
 * @param error_msg Error message string. Owned by SDK.
 * @param error_data Error payload. Specific to the error reported.
 * @param error_data_size Error payload size.
 */
typedef void (*CeptonSensorErrorCallback)(CeptonSensorHandle handle,
                                          int error_code, const char *error_msg,
                                          const void *error_data,
                                          size_t error_data_size);

/**
 * Returns 1 if sdk is initialized. 0 if not.
 */
CEPTON_EXPORT int CeptonIsInitialized(void);

/**
 * Initialize SDK.
 * Must be called before any other sdk functions.
 *
 * @param api_version `CEPTON_API_VERSION`, to ensure header and library match
 * @param cb This callback is unused, and is left here for API compatibility
 * purposes.
 */
CEPTON_EXPORT int CeptonInitialize(unsigned int api_version,
                                   CeptonSensorErrorCallback cb);

/**
 * Deinitialize SDK.
 */
CEPTON_EXPORT int CeptonDeinitialize(void);

/**
 * @brief Clear all point data in SDK pipeline.
 *
 * A few SDK modules keep point data in pipelines, such as AsyncRelay,
 * FrameAggregator and FrameFifo. This function guarantees that any future
 * listener callbacks will be invoked with new points received after the call
 * to this function.
 */
// CEPTON_EXPORT int CeptonClearPointDataPipeline(void);

/**
 * @brief Clear all cached sensors in SDK.
 *
 * Clear all cached SensorInfo inside SDK. Any sensor information reported after
 * this call will be based on new INFO packet received since the call. This can
 * be useful for a start of new session, or freshly after a sensor FW update.
 *
 * NOTE: Since INFO packet only arrive at fixed intervals (1Hz or 2Hz), there
 * will be a short time gap where point clouds doesn't have corresponding sensor
 * information, just like a fresh start from CeptonInitialize, or when a new
 * sensor is connected.
 */
// CEPTON_EXPORT int CeptonClearSensorInformationCache(void);

//------------------------------------------------------------------------------
// Network
//------------------------------------------------------------------------------

/**
 * Start networking. Use default port.
 */
CEPTON_EXPORT int CeptonStartNetworking(void);

/**
 * Start networking on a custom port.
 * @param port UDP port to connect to lidar. Use 8808 if lidar has the factory
 *  default port.
 */
CEPTON_EXPORT int CeptonStartNetworkingOnPort(uint16_t port);

/**
 * Start networking on a specific local network interface.
 * @param localIfAddress Host IP of the interface, use nullptr or "0.0.0.0" for
 * unspecified.
 * @param port Required, use 8808 if lidar has the factory default port.
 */
CEPTON_EXPORT int CeptonStartNetworkingUnicast(const char *localIfAddress,
                                               uint16_t port);

/**
 * Start networking by joining a multicast group.
 * @param targetMcastGroup Multicast group address.
 * @param localIfAddress Host IP of the interface, use nullptr or "0.0.0.0" for
 * unspecified.
 * @param port Required, use 8808 if lidar has the factory default port.
 */
CEPTON_EXPORT int CeptonStartNetworkingMulticast(const char *targetMcastGroup,
                                                 const char *localIfAddress,
                                                 uint16_t port);

/**
 * Stop networking.
 */
CEPTON_EXPORT int CeptonStopNetworking(void);

//------------------------------------------------------------------------------
// Replay
//------------------------------------------------------------------------------

/**
 * A simple set of APIs to replay pcap files.
 *
 * See samples/capture_replay on how to implement pause/resume/seek and
 * multiple concurrent replays functionalities.
 *
 * NOTE: CeptonReplayHandle refers to a capture being replayed. A capture can
 * have multiple sensors, therefore it is not corresponding to any SensorHandle.
 */
typedef uint64_t CeptonReplayHandle;
const uint32_t CEPTON_REPLAY_FLAG_LOAD_WITHOUT_INDEX = 1;
const uint32_t CEPTON_REPLAY_FLAG_PLAY_LOOPED = 2;
const uint32_t CEPTON_REPLAY_FLAG_LOAD_PAUSED = 4;

/**
 * Load pcap. By default, it will start indexing and plays async. Will need to
 * be unloaded in the end.
 * @param pcapFileName pcap file path for replay.
 * @param flags CEPTON_REPLAY_FLAG_LOAD_WITHOUT_INDEX,
 * CEPTON_REPLAY_FLAG_PLAY_LOOPED and CEPTON_REPLAY_FLAG_LOAD_PAUSED.
 * @param pHandle this will need to be passed in other replay calls.
 * @return CEPTON_SUCCESS or error code.
 *  CEPTON_ERROR_INVALID_STATE: too many loaded pcaps.
 *  CEPTON_ERROR_FILE_IO: load failed, file IO or format error.
 */
CEPTON_EXPORT int CeptonReplayLoadPcap(const char *pcapFileName, uint32_t flags,
                                       CeptonReplayHandle *pHandle);

/**
 * Unload the pcap.
 * @param {CeptonReplayHandle} pcap handle.
 */
CEPTON_EXPORT int CeptonReplayUnloadPcap(CeptonReplayHandle);

/**
 * Unload all loaded pcaps.
 */
CEPTON_EXPORT int CeptonReplayUnloadAll();

/**
 * Start playing. Resume from a paused state.
 * @param {CeptonReplayHandle} pcap handle.
 */
CEPTON_EXPORT int CeptonReplayPlay(CeptonReplayHandle);

/**
 * Play until the whole pcap is finished. API will fail if auto_replay is set.
 * This API can be called while playing. It will still block until whole
 * pcap is finished. If it is called from the player thread (e.g. points
 * callback), the API will fail.
 * @param {CeptonReplayHandle} pcap handle.
 */
CEPTON_EXPORT int CeptonReplayPlayToFinish(CeptonReplayHandle);

/**
 * Pause. Does nothing if already paused.
 * Listener Callback will not be called after return from this function.
 * @param {CeptonReplayHandle} pcap handle.
 */
CEPTON_EXPORT int CeptonReplayPause(CeptonReplayHandle);

/**
 * Get length of the pcap file.
 *
 * NOTE: If indexing is not done this call will block until it is.
 * It starts indexing if pcap was loaded with flag to not do indexing.
 * @param {CeptonReplayHandle} pcap handle.
 * @return length in microseconds of the pcap file. Error code if
 * return value is negative.
 */
CEPTON_EXPORT int64_t CeptonReplayGetLength(CeptonReplayHandle);

/**
 * Start indexing.
 *
 * This is useful when LoadPcap is called with WITHOUT_INDEX flag
 * If the indexing is already started this call has no effect.
 */
CEPTON_EXPORT int CeptonReplayStartIndexing(CeptonReplayHandle);

/**
 * Seek to position.
 *
 * NOTE: This call will block until indexing has been completed up to the
 * required seek position. The position is based on the recorded file and not
 * affected by the current playing speed.
 * @param {CeptonReplayHandle} pcap handle.
 * @param position microseconds from starting of the pcap.
 * @return CEPTON_SUCCESS or error code.
 */
CEPTON_EXPORT int CeptonReplaySeek(CeptonReplayHandle, int64_t position);

/**
 * Get the seek position.
 * @return microseconds from the start of the pcap if positive. Error code if
 * return value is negative.
 */
CEPTON_EXPORT int64_t CeptonReplayGetSeekPosition(CeptonReplayHandle);

/**
 * Play until next frame.
 * State will always be paused after this function returns.
 * @param {CeptonReplayHandle} pcap handle.
 * @return CEPTON_SUCCESS or error code.
 */
CEPTON_EXPORT int CeptonReplayNextFrame(CeptonReplayHandle);

/**
 * Sets whether the pcap should auto-replay when reaching the end.
 * This is equivalent to the flag CEPTON_REPLAY_FLAG_PLAY_LOOPED at load time
 * @param {CeptonReplayHandle} pcap handle.
 * @param {int} autoReplay 1 to replay automatically, 0 to not replay.
 */
CEPTON_EXPORT int CeptonReplaySetAutoReplay(CeptonReplayHandle, int autoReplay);

/**
 * Sets the speed for pcap replay.
 * @param {CeptonReplayHandle} pcap handle.
 * @param speed_percent 100 means 1x speed. Special value 0 for as fast as
 * possible.
 * @return CEPTON_SUCCESS or error code.
 */
CEPTON_EXPORT int CeptonReplaySetSpeed(CeptonReplayHandle, int speed);

/**
 * Gets replay speed.
 * @param {CeptonReplayHandle} pcap handle.
 * @return replay speed (100 mean 1x). Error code if return value is negative.
 */
CEPTON_EXPORT int CeptonReplayGetSpeed(CeptonReplayHandle);

/**
 * Gets last indexed position in microseconds from starting of pcap.
 * @param {CeptonReplayHandle} pcap handle.
 * @return last indexed position in microseconds. Error code if
 * return value is negative.
 */
CEPTON_EXPORT int64_t CeptonReplayGetIndexPosition(CeptonReplayHandle handle);

/**
 * Checks if async playback has finished.
 * @return 1 if playback is finished, 0 if not. Negative for error code.
 */
CEPTON_EXPORT int CeptonReplayIsFinished(CeptonReplayHandle handle);

/**
 * Check if playback is currently paused.
 * @return 1 if replay is paused, 0 if not paused, negative for error code.
 */
CEPTON_EXPORT int CeptonReplayIsPaused(CeptonReplayHandle handle);

//------------------------------------------------------------------------------
// Parser
//------------------------------------------------------------------------------

/**
 * Callback to handle incoming data. Return 0 to indicate the data is handled.
 *
 * @param handle Sensor handle.
 * @param timestamp Time data is received.
 * @param data Pointer to the data buffer.
 * @param data_size Size of the data buffer.
 * @param user_data User data passed during registration.
 * @returns 0 to indicate data is handled.
 */
typedef int (*CeptonParserCallback)(CeptonSensorHandle handle,
                                    int64_t timestamp, const uint8_t *data,
                                    size_t data_size, void *user_data);

/**
 * Register a parser. Unique parsers are determined by the pair of callback and
 * user_data. Save callback with different user_data are considered different
 * parsers.
 */
CEPTON_EXPORT int CeptonRegisterParser(CeptonParserCallback callback,
                                       void *user_data);

/**
 * Remove a parser. Only remove when both callback and user_data matches.
 */
CEPTON_EXPORT int CeptonUnregisterParser(CeptonParserCallback callback,
                                         void *user_data);

#define CEPTON_SDK_CONTROL_FLAG_PARSE_AMBIENT 0x0
#define CEPTON_SDK_CONTROL_FLAG_PARSE_TOF 0x1
CEPTON_EXPORT int CeptonSetSdkControlFlags(uint32_t flags);

//------------------------------------------------------------------------------
// Sensor Information
//------------------------------------------------------------------------------

/**
 * Callback to receive sensor information.
 * @param handle Sensor handle.
 * @param info Sensor information structure.
 * @param user_data User data passed during registration.
 */
typedef void (*CeptonSensorInfoCallback)(CeptonSensorHandle handle,
                                         const struct CeptonSensor *info,
                                         void *user_data);

/**
 * Register a callback to receive sensor information.
 */
CEPTON_EXPORT int CeptonListenSensorInfo(CeptonSensorInfoCallback callback,
                                         void *user_data);

/**
 * Remove a sensor information callback.
 */
CEPTON_EXPORT int CeptonUnlistenSensorInfo(CeptonSensorInfoCallback callback,
                                           void *user_data);

/**
 * Get number of sensors attached.
 * Use to check for new sensors. Sensors are not deleted until deinitialization.
 * @return 0 or positive for count, or negative to indicate error.
 */
CEPTON_EXPORT int CeptonGetSensorCount(void);

/**
 * Returns sensor information by sensor index.
 * Useful for getting information for all sensors.
 *
 * Returns error if index invalid.
 *
 * @param idx Sensor index.
 * @param info Sensor information structure to be filled.
 * @return 0 for success (info populated), negative for failure.
 */
CEPTON_EXPORT int CeptonGetSensorInformationByIndex(size_t idx,
                                                    struct CeptonSensor *info);

/**
 * Get sensor information by the handle
 * @param handle Sensor handle
 * @param info Sensor information structure to be filled.
 * @return 0 for success (info populated), negative for failure.
 */
CEPTON_EXPORT int CeptonGetSensorInformation(CeptonSensorHandle handle,
                                             struct CeptonSensor *info);

/**
 * Get sensor information by serial number
 * @param serial_number Sensor serial number
 * @param info Sensor information structure to be filled.
 * @return 0 for success (info populated), negative for failure.
 */
CEPTON_EXPORT int CeptonGetSensorInformationBySerialNumber(
    uint32_t serial_number, struct CeptonSensor *info);

//------------------------------------------------------------------------------
// Point Cloud Data
//------------------------------------------------------------------------------

// typedef void (*CeptonPointsCallback)(CeptonSensorHandle handle,
//                                      int64_t start_timestamp, size_t
//                                      n_points, size_t stride, const uint8_t
//                                      *points, void *user_data);

// CEPTON_EXPORT int CeptonListenPoints(CeptonPointsCallback callback,
//                                      void *user_data);

// CEPTON_EXPORT int CeptonUnlistenPoints(CeptonPointsCallback callback,
//                                        void *user_data);

/**
 * Callback to receive point cloud data.
 * @param handle Sensor handle.
 * @param start_timestamp Timestamp when the point cloud data starts.
 * @param n_points Number of points in the point cloud.
 * @param points Pointer to the point cloud data.
 * @param user_data User data passed during registration.
 */
typedef void (*CeptonPointsExCallback)(CeptonSensorHandle handle,
                                       int64_t start_timestamp, size_t n_points,
                                       const struct CeptonPointEx *points,
                                       void *user_data);
/**
 * Register a callback to receive point cloud data.
 */
CEPTON_EXPORT int CeptonListenPointsEx(CeptonPointsExCallback callback,
                                       void *user_data);

/**
 * Remove a point cloud data callback.
 */
CEPTON_EXPORT int CeptonUnlistenPointsEx(CeptonPointsExCallback callback,
                                         void *user_data);

/**
 * Aggregate mode definition:
 *  0 (default) for "natural" frames as defined by the sensor.
 *  Positive number enables timed aggregation with number of microseconds.
 *
 * NOTE: Any positive number above 1000 is allowed, but when frame exceeds
 * maximum number of points per frame, callback will be triggered even before
 * time limit has arrived.
 */
enum {
  CEPTON_AGGREGATION_MODE_NATURAL = 0,
  CEPTON_AGGREGATION_MODE_FIXED_10Hz = 100000,
  // CEPTON_AGGREGATION_MODE_FIXED_20Hz = 50000,
};

/**
 * Register a callback to receive point cloud data by frames.
 *
 * NOTE: For FIXED modes, every frame is complete. For NATURAL mode, the first
 * frame right after the first listener is registered collects only points up to
 * the first frame boundary and should not be considered a complete frame.
 * @return Error Code.
 *  CEPTON_ERROR_INVALID_STATE: if existing listeners are using a different
 *     aggregation mode.
 */
CEPTON_EXPORT int CeptonListenFramesEx(int aggregationMode,
                                       CeptonPointsExCallback callback,
                                       void *user_data);

/**
 * Remove a point cloud data frame callback.
 *
 * NOTE: The last unlisten call will destroy all frame buffers and stop frame
 * aggregation altogether. After that aggregationMode can be changed.
 */
CEPTON_EXPORT int CeptonUnlistenFramesEx(CeptonPointsExCallback callback,
                                         void *user_data);

//------------------------------------------------------------------------------
// Panic Message
//------------------------------------------------------------------------------

/**
 * Callback to receive panic messages from the sensor.
 * @param handle Sensor handle.
 * @param panic_packet Panic message structure.
 * @param user_data User data passed during registration.
 */
typedef void (*CeptonSensorPanicCallback)(
    CeptonSensorHandle handle, const struct CeptonPanicMessage *panic_packet,
    void *user_data);

/**
 * Register a callback to receive panic messages from the sensor.
 */
CEPTON_EXPORT int CeptonListenPanic(CeptonSensorPanicCallback callback,
                                    void *user_data);

/**
 * Remove a panic message callback.
 */
CEPTON_EXPORT int CeptonUnlistenPanic(CeptonSensorPanicCallback callback,
                                      void *user_data);

//------------------------------------------------------------------------------
// Frame FIFO Support (experimental)
//
// Frame FIFO is a standalone facility that listens to frames and keep them in
// a pre-allocated FIFO. With a frame FIFO in place, the application doesn't
// need to register callbacks to process frames asynchronously anymore. Instead
// application can have a simple structure to get frames one at a time. This is
// particularly suitable for off-line processing of captured data.
//
// - Frame FIFO is standalone. Implemented with callbacks internally.
// - Frame FIFO works only with frames, not point streams.
// - Frame FIFO queues frames from all connected sensors into a single FIFO.
//------------------------------------------------------------------------------

/**
 * Enables the frame FIFO for PointEx data.
 *
 * @return Error code.
 *  CEPTON_ERROR_INVALID_STATE: If frame FIFO is already enabled.
 *  CEPTON_ERROR_INVALID_STATE if existing listeners are using a different
 *     aggregation mode.
 */
CEPTON_EXPORT int CeptonEnableFrameFifoEx(int aggregationMode,
                                          unsigned frameCount);

/**
 * Disables the frame FIFO and discards all the buffered frames already in FIFO.
 *
 * @return Error code.
 *  CEPTON_ERROR_NOT_ENABLED: If the frame aggregation is not enabled, or
 *    if the current frame buffer is not released.
 */
CEPTON_EXPORT int CeptonDisableFrameFifoEx();

struct CeptonPointExData {
  CeptonSensorHandle handle;
  int64_t start_timestamp;
  size_t n_points;
  const struct CeptonPointEx *points;
};

/**
 * Get the pointer to the frame in FIFO.
 *
 * NOTE: This is only getting the pointer to the internal buffer. The buffers
 * are not copied. The buffers are valid until you call CeptonFrameFifoExNext()
 * move to the next frame in FIFO. All subsequent calls will return the same
 * data.
 *
 * @param pPointData Pointer of a CeptonPointExData instance.
 * @param maxWaitTime Maximum time to spend on waiting to get a frame before
 * returning an error code: CEPTON_ERROR_TIMEOUT. Value of -1 is interpreted as
 * waiting forever. Value of 0 will get frames already in FIFO without any
 * waiting time. Larger than 0 will be interpreted as the maximum waiting
 * time in millisecond.
 * @return Error code.
 *   CEPTON_ERROR_NOT_ENABLED: If the frame FIFO is not enabled.
 *   CEPTON_ERROR_INVALID_STATE: If last frame is not released.
 *   CEPTON_ERROR_TIMEOUT: If get frame timeout.
 *   CEPTON_ERROR_INVALID_ARGUMENTS: If pPointData is NULL.
 */
CEPTON_EXPORT int CeptonFrameFifoExGet(struct CeptonPointExData *pPointData,
                                       int maxWaitTime);

/**
 * Move frame FIFO to the next frame.
 *
 * This will free the current frame and invalidate the point returned by Get().
 * There is no waiting with this call, the waiting for next frame will happen
 * when the Get() function is called again.
 *
 * @return Error code.
 *   CEPTON_ERROR_NOT_ENABLED: If the frame FIFO is not enabled.
 *   CEPTON_ERROR_INVALID_STATE: If no frame buffer is acquired.
 */
CEPTON_EXPORT int CeptonFrameFifoExNext();

/**
 * Check if the FIFO is empty. Equivalent to Size() > 0.
 *
 * @return 1 if FIFO is empty. 0 if not empty. Negative for error
 *   CEPTON_ERROR_NOT_ENABLED: If frame FIFO is not enabled.
 */
CEPTON_EXPORT int CeptonFrameFifoExEmpty();

/**
 * Return the number of frames in FIFO right now.
 *
 * @return Larger or equal to 0 for fifo usage. Negative for error
 *   CEPTON_ERROR_NOT_ENABLED: If frame FIFO is not enabled.
 */
CEPTON_EXPORT int CeptonFrameFifoExSize();

/**
 * Check if the FIFO is full. Equivalent to Size() == frame_count.
 * @return 1 if FIFO is full. 0 if not full. Negative for error
 *   CEPTON_ERROR_NOT_ENABLED: If frame FIFO is not enabled.
 */
CEPTON_EXPORT int CeptonFrameFifoExFull();

//-----------------------------------------------------------------------------
// PCAP Writer Support (experimental)
//
// PCAP writer register a UDP data parser and use it to dump the UDP packet to
// the current open PCAP file. All the received UDP packets are dumped without
// any filtering to be future proof.
//
// PCAP files captured by PCAP writer are guaranteed to work with SDK's PCAP
// replay functionalities.
//
// CAVEATS:
//  - Due to the nature of UDP networking stack, application layer won't see
//    the MAC of lidar. We fill in the source MAC with Cepton's default
//    18-12-12-xx-xx-xx where the last 3 bytes are the lower 3 bytes calculated
//    by (IP - 0xC0A82020).
//  - All VLAN header info, if exists, will be lost.
//  - Fragmented IP packets will be written as jumbo frames at the time of last
//    fragments arrival.
//-----------------------------------------------------------------------------

/**
 * @brief Open PCAP file for writing
 *
 * There can only be one open PCAP file. Attempt to open again will result in
 * error. SDK will hold an open file handle until Close() is called.
 *
 * The recording only starts with a Start() call. Right after Open() the
 * automatic dumping is not enabled by default. This enables application where
 * it is desirable to record at a frame boundary or other critical events. Or
 * for some app, dumping happens only explicitly with calls to WriteUdpPacket().
 *
 * @param pcapFile File name to write to. If file exists it will be truncated
 * @return Error code.
 *  CEPTON_ERROR_INVALID_STATE: If CeptonPcapWriterOpen is already called.
 *  CEPTON_ERROR_FILE_IO: If opening or writing PCAP header failed.
 */
// CEPTON_EXPORT int CeptonPcapWriterOpen(char const *pcapFile);

/**
 * @brief Stop the current PCAP file being written to.
 *
 * @return Error code.
 *  CEPTON_ERROR_NOT_ENABLED: If CeptonPcapWriterOpen is not called.
 *  CEPTON_ERROR_FILE_IO: If closing the file failed.
 */
// CEPTON_EXPORT int CeptonPcapWriterClose();

/**
 * @brief Start automatic dumping of UDP packets.
 *
 * @return Error code.
 *  CEPTON_ERROR_NOT_ENABLED: If no PCAP is open.
 *  CEPTON_ERROR_INVALID_STATE: If dumping is already started.
 */
// CEPTON_EXPORT int CeptonPcapWriterStart();

/**
 * @brief Stop automatic dumping of UDP packets.
 *
 * With automatic dumping stopped, the file is still open until a Close() call.
 * Data can still be dumped into the PCAP with explicit WritePcapPacket() calls
 *
 * @return Error code.
 *  CEPTON_ERROR_NOT_ENABLED: If no PCAP is open.
 *  CEPTON_ERROR_INVALID_STATE: If dumping is not started.
 */
// CEPTON_EXPORT int CeptonPcapWriterStop();

/**
 * @brief Explicitly write a UDP packet into the PCAP file.
 *
 * @param data UDP payload data.
 * @param dataLen Length in bytes.
 * @param timestamp Pass 0 to use current system timestamp. This timestamp
 *  goes to PCAP record header.
 * @return Error code.
 *  CEPTON_ERROR_NOT_ENABLED: If CeptonPcapWriterOpen is not called.
 *  CEPTON_ERROR_FILE_IO: If writing to file failed.
 *  CEPTON_ERROR_INVALID_ARGUMENTS: If dataLen exceeds maximum UDP payload size.
 */
// CEPTON_EXPORT int CeptonPcapWriterWriteUdpPacket(CeptonSensorHandle h,
//                                                  uint8_t const *data,
//                                                  size_t dataLen,
//                                                  int64_t timestamp);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CEPTON_SDK3_H
