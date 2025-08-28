#pragma once

#include <exception>

#if defined(WIN32)
#include <windows.h>

#include <codecvt>
#else
#include <dlfcn.h>
#include <libgen.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#include <string>
#endif

#include <cepton_sdk2.h>

class ProxyLoader {
 public:
  class ModuleLoadError : public std::exception {
    char const* msg;

   public:
    ModuleLoadError(char const* what) : msg(what) {}
    char const* what() const noexcept override { return msg; }
  };

  class SymbolLoadError : public std::exception {
   private:
    char const* symbol;

   public:
    SymbolLoadError(char const* name) : symbol(name) {}
    char const* Symbol() const { return symbol; }
  };

 private:
  char str_buffer[256] = {0};

 protected:
  typedef int (*FunctionHandle)();
  typedef void* ModuleHandle;

  ModuleHandle sys_handle = nullptr;
  bool all_check_passed   = false;

  ProxyLoader() = default;
  ~ProxyLoader() {
    if (sys_handle) CloseModule();
  }

  static void dummyFunction() {}

  // Loading of shared library
  void OpenModule(char const* shared_lib_name) {
    std::string file_name;
#if defined(WIN32)
    file_name = shared_lib_name;
    HMODULE sdk2;
    if (GetModuleHandleEx(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                              GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          (LPCSTR)&dummyFunction, &sdk2)) {
      char* sdk2Path = (char*)malloc(4096);  // PATH_MAX is tricky
      if (sdk2Path) {
        DWORD len = GetModuleFileName(sdk2, sdk2Path, 4096);
        if (len && len < 4096) {
          for (; len; len--)
            if (sdk2Path[len] == '\\') break;
          if (len) {
            len++;  // Move to the first char to overwrite
            strcpy_s(sdk2Path + len, 4096 - len, shared_lib_name);
            file_name = sdk2Path;
          }
        }
        free(sdk2Path);
      }
    }

    sys_handle = LoadLibrary(file_name.c_str());
#else  // APPLE or UNIX
#if APPLE
    file_name = std::string("lib") + shared_lib_name + ".dylib";
#else
    file_name = std::string("lib") + shared_lib_name + ".so";
#endif
    Dl_info info;
    if (dladdr((const void*)&dummyFunction, &info)) {
      char const* sdk2Path = info.dli_fname;
      size_t len           = strlen(info.dli_fname);
      for (; len; len--)
        if (sdk2Path[len] == '/') break;
      if (len) {
        file_name = std::string(sdk2Path, sdk2Path + len + 1) + file_name;
      }
    }
    // Deepbind is ONLY to work around an asio bug, pending some better fix.
    // Seems to be a known version with a particular DDS
    // https://github.com/eProsima/Fast-DDS/issues/1484
    sys_handle = dlopen(file_name.c_str(), RTLD_LAZY | RTLD_DEEPBIND);
#endif
  }

  void CloseModule() {
#if defined(WIN32)
    FreeLibrary((HMODULE)sys_handle);
#else
    dlclose(sys_handle);
#endif
    sys_handle = nullptr;
  }

  FunctionHandle LoadFunction(char const* function_name) {
#if defined(WIN32)
    return (FunctionHandle)GetProcAddress((HMODULE)sys_handle, function_name);
#else
    return (FunctionHandle)dlsym(sys_handle, function_name);
#endif
  }

  char const* ErrorMessage() {  // NOLINT
#if defined(WIN32)
    DWORD dw = GetLastError();
    FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                  NULL, dw, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                  str_buffer, sizeof(str_buffer), NULL);
    return str_buffer;
#else
    return dlerror();
#endif
  }

  template <typename FT>
  void Load(FT& fc, char const* fn, bool optional = false) {
    auto f = LoadFunction(fn);
    if (!optional && f == nullptr) {
      throw SymbolLoadError(fn);
    }
    fc = (FT)f;
  }

  virtual void LoadSymbols() = 0;
  virtual void UnloadSymbols() {}
  virtual bool CheckCaps() { return true; }  // Return false to fail the load

 public:
  bool supported() const { return sys_handle != nullptr && all_check_passed; }

  void LoadModule(char const* name) {
    OpenModule(name);
    if (sys_handle == nullptr) {
      throw ModuleLoadError(ErrorMessage());
    }
    LoadSymbols();
    all_check_passed = CheckCaps();
  }

  void UnloadModule() {
    UnloadSymbols();
    CloseModule();
  }
};

class SdkProxy : public ProxyLoader {
  typedef int (*ft_initialize)(int ver, CeptonSensorErrorCallback cb);
  typedef int (*ft_deinitialize)();

  typedef int (*ft_receive)(CeptonSensorHandle handle, int64_t timestamp,
                            const uint8_t* const buffer, size_t buffer_size);

  typedef int (*ft_listen)(int aggregation_mode, CeptonPointsCallback cb,
                           void* const user_data);
  typedef int (*ft_unlisten)();

  typedef int (*ft_listen_info)(CeptonSensorInfoCallback cb, void* user_data);
  typedef int (*ft_unlisten_info)();

  typedef int (*ft_listen_panic)(CeptonSensorPanicCallback cb, void* user_data);
  typedef int (*ft_unlisten_panic)();

  typedef int (*ft_enable_legacy)();
  typedef int (*ft_replay_load_pcap)(const char* pcap_file, uint32_t flag,
                                     CeptonReplayHandle* handle);
  typedef int (*ft_start_networking_port)(uint16_t port);

  ft_initialize fpInitialize                     = nullptr;
  ft_deinitialize fpDeinitialize                 = nullptr;
  ft_receive fpReceive                           = nullptr;
  ft_listen fpListen                             = nullptr;
  ft_unlisten fpUnlisten                         = nullptr;
  ft_listen_info fpListenInfo                    = nullptr;
  ft_unlisten_info fpUnlistenInfo                = nullptr;
  ft_listen_panic fpListenPanic                  = nullptr;
  ft_unlisten_panic fpUnlistenPanic              = nullptr;
  ft_enable_legacy fpEnableLegacy                = nullptr;
  ft_replay_load_pcap fpReplayLoadPcap           = nullptr;
  ft_start_networking_port fpStartNetworkingPort = nullptr;
  void LoadSymbols() final {
    Load(fpInitialize, "CeptonInitialize");
    Load(fpDeinitialize, "CeptonDeinitialize");
    Load(fpListen, "CeptonListenFrames");
    Load(fpUnlisten, "CeptonUnlistenFrames");
    Load(fpListenInfo, "CeptonListenSensorInfo");
    Load(fpUnlistenInfo, "CeptonUnlistenSensorInfo");
    Load(fpListenPanic, "CeptonListenPanic");
    Load(fpUnlistenPanic, "CeptonUnlistenPanic");
    Load(fpEnableLegacy, "CeptonEnableLegacyTranslation");
    Load(fpReplayLoadPcap, "CeptonReplayLoadPcap");
    Load(fpStartNetworkingPort, "CeptonStartNetworkingOnPort");
  }

 public:
  int Initialize() { return fpInitialize(CEPTON_API_VERSION, nullptr); }
  int Deinitialize() { return fpDeinitialize(); }

  int ListenFrames(int aggregation_mode, CeptonPointsCallback cb,
                   void* user_data) {
    return fpListen(aggregation_mode, cb, user_data);
  }
  int Unlisten() { return fpUnlisten(); }

  int ListenInfo(CeptonSensorInfoCallback cb, void* user_data) {
    return fpListenInfo(cb, user_data);
  }
  int UnlistenInfo() { return fpUnlistenInfo(); }

  int ListenPanic(CeptonSensorPanicCallback cb, void* user_data) {
    return fpListenPanic(cb, user_data);
  }
  int UnlistenPanic() { return fpUnlistenPanic(); }

  int StartNetworkingOnPort(uint16_t port) {
    return fpStartNetworkingPort(port);
  }

  int EnableLegacy() { return fpEnableLegacy(); }

  int ReplayLoadPcap(const char* pcap_file, uint32_t flag,
                     CeptonReplayHandle* handle) {
    return fpReplayLoadPcap(pcap_file, flag, handle);
  }
};
