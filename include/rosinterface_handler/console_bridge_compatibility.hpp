#pragma once

// delete deprecated, conflicting macros lodDebug, logInfo, logWarn, logError
// see also https://github.com/ros/class_loader/pull/66
#ifdef CLASS_LOADER__CONSOLE_BRIDGE_COMPATIBILITY_HPP_
#undef logDebug
#undef logInfo
#undef logWarn
#undef logError
#endif
