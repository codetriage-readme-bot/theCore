#ifndef KE02_PLATFORM_CONSOLE_HPP_
#define KE02_PLATFORM_CONSOLE_HPP_

extern "C" {
#include <io.h>
}

namespace ecl
{

//! Bypasses console drivers and puts data directly to the UART
static inline void bypass_putc(char c)
{
    out_char(c);
}

} // namespace ecl


#endif // KE02_PLATFORM_CONSOLE_HPP_
