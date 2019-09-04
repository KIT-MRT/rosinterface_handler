#pragma once

// delete deprecated, conflicting macros lodDebug, logInfo, logWarn, logError
// see also https://github.com/ros/class_loader/pull/66
#ifdef CLASS_LOADER__CONSOLE_BRIDGE_COMPATIBILITY_HPP_
#ifdef logDebug
// otherwise the function name in the Interface.h.template is replaced by the macro
#undef logDebug
#endif
#ifdef logInfo
// otherwise the function name in the Interface.h.template is replaced by the macro
#undef logInfo
#endif
#ifdef logWarn
// otherwise the function name in the Interface.h.template is replaced by the macro
#undef logWarn
#endif
#ifdef logError
// otherwise the function name in the Interface.h.template is replaced by the macro
#undef logError
#endif
#endif
