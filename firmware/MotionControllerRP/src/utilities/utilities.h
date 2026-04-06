// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include <string>
#include <vector>

//*** FUNCTIONS *************************************************************************

std::vector<std::string> get_file_list(const char* dirname, bool include_dirs);

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ return "FORMAT_ERROR"; }
    auto size = static_cast<size_t>( size_s );
    std::string buf( size, '\0' );
    std::snprintf( &buf[0], size, format.c_str(), args ... );
    return std::string( buf.begin(), buf.end() - 1 ); // We don't want the '\0' inside
}
