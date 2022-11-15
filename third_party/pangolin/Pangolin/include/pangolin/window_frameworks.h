// CMake generated file. Do Not Edit.

#pragma once

namespace pangolin {

void RegisterX11WindowFactory();

inline bool LoadBuiltInWindowFrameworks()
{
    RegisterX11WindowFactory();
    return true;
}

} // pangolin
