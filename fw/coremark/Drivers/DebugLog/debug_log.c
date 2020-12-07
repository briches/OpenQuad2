///////////////////////////////////////////////////////////////////////
// Copyright © 2020,
// 4iiii Innovations Inc.,
// Cochrane, Alberta, Canada.
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are not permitted without express written approval of
// 4iiii Innovations Inc.
///////////////////////////////////////////////////////////////////////

#include "debug_log.h"

#ifdef FREERTOS
#include "cmsis_os.h"
#else
#include "stm32h7xx_hal.h"
#endif

// String table of module names correlating to the module_id_t enumeration.
const char *module_stringtable[] = 
{
   "main",
};

// Temporary buffer used for printing routines
static uint8_t line_buffer[32];

// Flag indicating the enabled or disabled state of default timestamp printing
static bool timestamp_enabled = true;

// Local (and arguably safer) implementation of vprintf
void _vprintf(const char *format, va_list ap);

// Data field printing routines ------------------------------------------------------------------
//
//      These routines are used to write a data field to the output buffer. All of these use
//      _putc(..) to perform their actual writing, in case future development requires hooking
//      into the output of the debug module.

int _putx(uint64_t integer, char hex_base, int width, char pad);
int _putu(uint64_t integer, int width, char pad);
int _puti(int64_t integer, int width, char pad);
int _putf(double value, int width, int precision, char pad);
int _puts(const char *p_string);
int _putc(char byte);

// Externals

void init_rtt_logging(void)
{
   // woo hoo!
   SEGGER_RTT_Init();
}

/** 
 * @name debug_print_timestamp
 * @brief: Print the timestamp in microseconds to debug output
 *
 * Parameter:
 *    enable:     if true, set the behavior to print timestamp before buffer
 */
static void debug_print_timestamp()
{
   uint32_t decimal;
   uint32_t intpart;

   #ifdef FREERTOS
      uint32_t time = osKernelGetTickCount();
   #else
      uint32_t time = HAL_GetTick();
   #endif
   intpart = (uint32_t)(time / 1000);
   decimal = (uint32_t)(time - (uint32_t)(1000 * intpart));

   _putu(intpart, 1, ' ');
   _putc('.');
   _putu(decimal, 3, '0');
   _putc('s');
}

/** 
 * @name debug_timestamp_enable
 * @brief: Set the behavior of debug_printf to print timestamp before the buffer
 *
 * Parameter:
 *    enable:     if true, set the behavior to print timestamp before buffer
 */
void debug_timestamp_enable(bool enable)
{
   timestamp_enabled = enable;
}

/** 
 * @name debug_error
 * @brief: Print/append to the last debug message
 *
 * Parameter:
 *    p_format        Pointer to the printf(..)-styled format string
 *    ...             Variable argument list, same as the classic printf(..)
 */
void debug_error(module_id_t source, const char *p_format, ...)
{
   // Change the color code to RED
   _puts(VT100_TEXT_BRIGHT_RED);

   // Write the line header
   debug_print_timestamp();

   // Print the source module name
   _puts(" [");
   if (source >= NUM_MODULES || source < 0)
      _puts("???");
   else
      _puts(module_stringtable[source]);
   _putc(']');
   _putc(' ');

   // Write the body of data
   va_list ap;
   va_start(ap, p_format);
   _vprintf(p_format, ap);
   va_end(ap);

   // Change the color code back to default
   _puts(VT100_RESET);

   // Finish up
   _puts("\r\n");
}

/** 
 * @name debug_printf
 * @brief: Print a debug message, with a timestamp.
 *
 * Parameter:
 *    p_format        Pointer to the printf(..)-styled format string
 *    ...             Variable argument list, same as the classic printf(..)
 */
void debug_eeprintf(const char *p_format, ...)
{
   // Print the timestamp
   if (timestamp_enabled)
      debug_print_timestamp();

   _putc(' ');

   // Write the body of data
   va_list ap;
   va_start(ap, p_format);
   _vprintf(p_format, ap);
   va_end(ap);
}

/** 
 * @name debug_printf
 * @brief: Print a debug message, with a timestamp.
 *
 * Parameter:
 *    p_format        Pointer to the printf(..)-styled format string
 *    ...             Variable argument list, same as the classic printf(..)
 */
void debug_printf(module_id_t source, const char *p_format, ...)
{

   if (source == DEBUG_YELLOW_HIGHLIGHT_SELECT)
      _puts(VT100_TEXT_BRIGHT_YELLOW);
   else if (source == DEBUG_CYAN_HIGHLIGHT_SELECT)
      _puts(VT100_TEXT_BRIGHT_CYAN);

   // Print the timestamp
   if (timestamp_enabled)
      debug_print_timestamp();

   // Print the source module name
   _puts(" [");
   if (source >= NUM_MODULES || source < 0)
      _puts("???");
   else
      _puts(module_stringtable[source]);
   _putc(']');
   _putc(' ');

   // Write the body of data
   va_list ap;
   va_start(ap, p_format);
   _vprintf(p_format, ap);
   va_end(ap);

   // Finish up
   _puts("\r\n");

   // Change the color code back to default
   if (source == DEBUG_YELLOW_HIGHLIGHT_SELECT || source == DEBUG_CYAN_HIGHLIGHT_SELECT)
      _puts(VT100_RESET);
}

/** 
 * @name debug_print_buffer( const uint8_t * p_buffer, int length, 
                         uint32_t start_address, int columns )
 * @brief: Print a debug message, with a timestamp.
 *
 * Parameter:
 *      source          Identifies the module of code that is generating the message
 *
 *      p_buffer        Pointer to the buffer to print
 *
 *      length          Size of the buffer, in bytes
 *
 *      start_address   Starting address to display on the index
 *
 *      columns         Number of bytes to display per row
 */
void debug_print_buffer(module_id_t source, const uint8_t *p_buffer, unsigned int length,
                        uint32_t start_address, int columns)
{
   if (source == DEBUG_YELLOW_HIGHLIGHT_SELECT)
      _puts(VT100_TEXT_BRIGHT_YELLOW);
   else if (source == DEBUG_CYAN_HIGHLIGHT_SELECT)
      _puts(VT100_TEXT_BRIGHT_CYAN);

   // Draw the top axis
   // _puts("\r\n  Address |");
   for (int i = 0; i < columns; i++)
   {
      _puts("   ");
      _putx(i, 'A', 2, '0');
   }
   _puts("\r\n");

   // _puts("----------+");
   for (int i = 0; i < columns; i++)
      _puts("-----");
   _puts("-\r\n");

   while (length > 0)
   {
      // Print the left axis
      // _putc(' ');
      // _putx(start_address, 'A', 8, '0');
      // _puts(" | ");

      // Print a row's worth of data
      for (int i = 0; i < columns; i++)
      {
         if (length > 0)
         {
            _puts(" 0x");
            _putx(*(p_buffer++), 'A', 2, '0');
            length--;
         }
         else
         {
            _puts("  ");
         }
      }

      // End the line
      _puts("\r\n");
      start_address += columns;
   }

   _puts("\r\n");

   // Change the color code back to default
   if (source == DEBUG_YELLOW_HIGHLIGHT_SELECT || source == DEBUG_CYAN_HIGHLIGHT_SELECT)
      _puts(VT100_RESET);
}

/** 
 * @name   void _vprintf( const char *format, va_list ap )
 *
 *  @brief:
 *      Internal implementation of the C standard vprintf(..) routine which performs a formatted
 *      print to the output buffer.
 *
 *  Parameter:
 *      format          The formwat string
 *      ap              Variable argument list of parameters for the print
 */
void _vprintf(const char *format, va_list ap)
{
   int width;
   int precision;
   char pad;
   bool longlongint;

   for (; *format != '\0'; format++)
   {
      // Just print the character unless it's an escape sequence
      if (*format != '%')
      {
         _putc(*format);
      }
      // ..otherwise, handle the escape sequence
      else
      {
         format++;
         width = 0;
         precision = 6;
         pad = '\0';
         longlongint = false;

         // Abort if the string ended
         if (*format == '\0')
            break;

         // Double escape sequence: print '%' instead
         if (*format == '%')
         {
            _putc(*format);
            continue;
         }

         // Switch to right padding if the escape sequence starts with '-'
         if (*format == '-')
         {
            format++;
            pad = ' ';
         }

         // Pad to zero
         if (*format == '0')
         {
            format++;
            pad = '0';
         }

         // Process any variable-width specification
         if (*format == '*')
         {
            width = va_arg(ap, int);
            format++;
         }
         // ..otherwise, process any hard coded width specification
         else
         {
            for (; *format >= '0' && *format <= '9'; format++)
               width = (width * 10) + (*format - '0');
         }

         // Process any precision specification
         if (*format == '.')
         {
            format++;
            precision = 0;
            for (; *format >= '0' && *format <= '9'; format++)
               precision = (precision * 10) + (*format - '0');
         }

         // Process any length designation: 'l' is implicity supported, 'll' designates 64-bit (long-long int)
         if (*format == 'l')
         {
            format++;
            if (*format == 'l')
            {
               format++;
               longlongint = true;
            }
         }

         // Print a string
         if (*format == 's')
         {
            char *p_string = va_arg(ap, char *);
            int chars = _puts(p_string);
            while (chars++ < width)
               _putc(' ');
         }
         // ..print a signed integer
         else if (*format == 'd')
         {
            if (longlongint)
            {
               int64_t integer = va_arg(ap, int64_t);
               _puti(integer, width, pad);
            }
            else
            {
               int32_t integer = va_arg(ap, int32_t);
               _puti(integer, width, pad);
            }
         }
         // ..print an unsigned integer
         else if (*format == 'u')
         {
            if (longlongint)
            {
               uint64_t integer = va_arg(ap, uint64_t);
               _putu(integer, width, pad);
            }
            else
            {
               uint32_t integer = va_arg(ap, uint32_t);
               _putu(integer, width, pad);
            }
         }
         // ..print a character
         else if (*format == 'c')
         {
            char c = va_arg(ap, int);
            _putc(c);
         }
         // ..print lower-case hexadecimal characters
         else if (*format == 'x')
         {
            if (longlongint)
            {
               uint64_t integer = va_arg(ap, uint64_t);
               _putx(integer, 'a', width, pad);
            }
            else
            {
               uint32_t integer = va_arg(ap, uint32_t);
               _putx(integer, 'a', width, pad);
            }
         }
         // ..print upper-case hexadecimal characters
         else if (*format == 'X')
         {
            if (longlongint)
            {
               uint64_t integer = va_arg(ap, uint64_t);
               _putx(integer, 'A', width, pad);
            }
            else
            {
               uint32_t integer = va_arg(ap, uint32_t);
               _putx(integer, 'A', width, pad);
            }
         }
         // ..print floating point value
         else if (*format == 'f')
         {
            double value = va_arg(ap, double);
            _putf(value, width, precision, pad);
         }
         // ..abort the print if we've reached the end of the format string
         else if (*format == '\0')
         {
            return;
         }
         // ..print lower-case hex dump of an array (NON-STANDARD FORMAT SPECIFIER)
         else if (*format == 'b')
         {
            if (width == 0)
               width = 1;

            uint8_t *p_buffer = (uint8_t *)va_arg(ap, void *);
            _putx((int)*(p_buffer++), 'a', 2, '0');
            while (--width)
            {
               _putc(' ');
               _putx((int)*(p_buffer++), 'a', 2, '0');
            }
         }
         // // ..print upper-case hex dump of an array (NON-STANDARD FORMAT SPECIFIER)
         // else if (*format == 'B')
         // {
         //    if (width == 0)
         //       width = 1;

         //    uint8_t *p_buffer = (uint8_t *)va_arg(ap, void *);
         //    _putx((int)*(p_buffer++), 'A', 2, '0');
         //    while (--width)
         //    {
         //       _putc(' ');
         //       _putx((int)*(p_buffer++), 'A', 2, '0');
         //    }
         // }
         // // ..print a pointer to a generic_data_t instance (NON-STANDARD FORMAT SPECIFIER)
         // else if (*format == '#')
         // {
         //    int chars = 0;
         //    generic_data_t *p_data = (generic_data_t *)va_arg(ap, void *);
         //    switch (p_data->type)
         //    {
         //    case typeUNDEFINED:
         //       chars = _puts("(undefined)");
         //       break;
         //    case typeBL:
         //       if (p_data->value.bl)
         //          chars = _puts("true");
         //       else
         //          chars = _puts("false");
         //       break;
         //    case typeU8:
         //       chars = _putu(p_data->value.u8, width, pad);
         //       break;
         //    case typeU16:
         //       chars = _putu(p_data->value.u16, width, pad);
         //       break;
         //    case typeU32:
         //       chars = _putu(p_data->value.u32, width, pad);
         //       break;
         //    case typeU64:
         //       chars = _putu(p_data->value.u64, width, pad);
         //       break;
         //    case typeI8:
         //       chars = _puti(p_data->value.u8, width, pad);
         //       break;
         //    case typeI16:
         //       chars = _puti(p_data->value.u16, width, pad);
         //       break;
         //    case typeI32:
         //       chars = _puti(p_data->value.u32, width, pad);
         //       break;
         //    case typeI64:
         //       chars = _puti(p_data->value.u64, width, pad);
         //       break;
         //    case typeFLOAT:
         //       chars = _putf(p_data->value.fl, width, precision, pad);
         //       break;
         //    case typeDOUBLE:
         //       chars = _putf(p_data->value.db, width, precision, pad);
         //       break;
         //    case typeBYTES:
         //    {
         //       chars = _putx((int)p_data->value.bytes[0], 'A', 2, '0');
         //       for (width = 1; width < 8; width++)
         //       {
         //          chars += _putc(' ');
         //          chars += _putx((int)p_data->value.bytes[width], 'A', 2, '0');
         //       }
         //       break;
         //    }
         //    default:
         //       chars += _puts("(invalid)");
         //       break;
         //    }

         //    while (chars++ < width)
         //       _putc(' ');
         // }
         // // ..print the type for a generic_data_t instance (NON-STANDARD FORMAT SPECIFIER)
         // else if (*format == '^')
         // {
         //    int chars;
         //    generic_data_t *p_data = (generic_data_t *)va_arg(ap, void *);
         //    switch (p_data->type)
         //    {
         //    case typeUNDEFINED:
         //       chars = _puts("unknown");
         //       break;
         //    case typeBL:
         //       chars = _puts("boolean");
         //       break;
         //    case typeU8:
         //       chars = _puts("uint8");
         //       break;
         //    case typeU16:
         //       chars = _puts("uint16");
         //       break;
         //    case typeU32:
         //       chars = _puts("uint32");
         //       break;
         //    case typeU64:
         //       chars = _puts("uint64");
         //       break;
         //    case typeI8:
         //       chars = _puts("int8");
         //       break;
         //    case typeI16:
         //       chars = _puts("int16");
         //       break;
         //    case typeI32:
         //       chars = _puts("int32");
         //       break;
         //    case typeI64:
         //       chars = _puts("int64");
         //       break;
         //    case typeFLOAT:
         //       chars = _puts("float");
         //       break;
         //    case typeDOUBLE:
         //       chars = _puts("double");
         //       break;
         //    case typeBYTES:
         //       chars = _puts("bytes");
         //       break;
         //    default:
         //       chars = _puts("invalid");
         //       break;
         //    }

         //    while (chars++ < width)
         //       _putc(' ');
         // }
         // ..otherwise, ignore the escape sequence
         else
         {
            continue;
         }
      }
   }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Internal Routine:   int _putc( char byte )
//
//  Description:
//
//      Internal utility routine which prints a single character to the output buffer. If we're
//      a DEBUG build (aka made using the 'make debug' command) then the character is also
//      output over RTT.
//
//  Parameter:
//
//      byte            The character to print
//
///////////////////////////////////////////////////////////////////////////////////////////////////
int _putc(char byte)
{
   // #ifdef DEBUG
   SEGGER_RTT_Write(0, &byte, 1);
   // #endif

   return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Internal Routine:   int _puts( const char *p_string )
//
//  Description:
//
//      Internal utility routine which prints a string to the output buffer.
//
//  Parameter:
//
//      p_string        Pointer to the string to print
//
///////////////////////////////////////////////////////////////////////////////////////////////////
int _puts(const char *p_string)
{
   int i;
   for (i = 0; *p_string != '\0'; i++)
      _putc(*(p_string++));

   return i;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Internal Routine:   int _puti( int64_t integer, int width, char pad )
//
//  Description:
//
//      Internal utility routine which prints a signed integer to the output buffer.
//
//  Parameter:
//
//      integer         The value to print
//
//      width           The minimum number of characters to use for this print
//
//      pad             The padding character to use to fill unused space
//
///////////////////////////////////////////////////////////////////////////////////////////////////
int _puti(int64_t integer, int width, char pad)
{
   // Mark if the integer is negative
   bool negative = integer < 0;
   if (negative)
      integer = integer * -1;

   // Convert the integer into a string (in reverse order)
   int length = 0;
   uint8_t *p_insert = line_buffer;
   do
   {
      *(p_insert++) = (integer % 10) + '0';
      integer /= 10;
      length++;
   } while (integer != 0);

   // Print the string back, re-use the 'integer' variable for this
   integer = length;
   if (negative)
   {
      _putc('-');
      length++;
   }
   while (width > length++)
      _putc(pad);
   while (integer--)
      _putc(*(--p_insert));

   // Finished, return the length
   return length - 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Internal Routine:   int _putu( uint64_t integer, int width, char pad )
//
//  Description:
//
//      Internal utility routine which prints an unsigned integer to the output buffer.
//
//  Parameter:
//
//      integer         The value to print
//
//      width           The minimum number of characters to use for this print
//
//      pad             The padding character to use to fill unused space
//
///////////////////////////////////////////////////////////////////////////////////////////////////
int _putu(uint64_t integer, int width, char pad)
{
   // Convert the integer into a string (in reverse order)
   int length = 0;
   uint8_t *p_insert = line_buffer;
   do
   {
      *(p_insert++) = (integer % 10) + '0';
      integer /= 10;
      length++;
   } while (integer != 0);

   // Print the string back, re-use the 'integer' variable for this
   integer = length;
   while (width > length++)
      _putc(pad);
   while (integer--)
      _putc(*(--p_insert));

   // Finished, return the length
   return length - 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Internal Routine:   int _putx( uint64_t integer, char hex_base, int width, char pad )
//
//  Description:
//
//      Internal utility routine which prints an unsigned integer to the output buffer in hex.
//
//  Parameter:
//
//      integer         The value to print
//
//      hex_base        The base character to use for digits after '9': use 'a' for lower-case
//                      hex values (aka 0xdeadbeef), use 'A' for upper-case hex values (aka
//                      0xDEADBEEF)
//
//      width           The minimum number of characters to use for this print
//
//      pad             The padding character to use to fill unused space
//
///////////////////////////////////////////////////////////////////////////////////////////////////
int _putx(uint64_t integer, char hex_base, int width, char pad)
{
   // Convert the integer into a string (in reverse order)
   int length;
   uint8_t *p_insert = line_buffer;
   for (length = 0; integer != 0; length++, p_insert++)
   {
      *p_insert = (integer & 15) + '0';
      if (*p_insert > '9')
         *p_insert += hex_base - '9' - 1;

      integer >>= 4;
   }

   // Print the string back, re-use the 'integer' variable for this
   while (width-- > length)
      _putc(pad);
   integer = length;
   while (integer--)
      _putc(*(--p_insert));

   // Finished, return the length
   return length;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Internal Routine:   int _putf( double value, int width, int precision, char pad )
//
//  Description:
//
//      Internal utility routine which prints a floating point value to the output buffer.
//
//  Parameter:
//
//      value           The value to print
//
//      width           The minimum number of characters to use for this print
//
//      precision       The number of digits to use for the fractional component
//
//      pad             The padding character to use to fill unused space
//
///////////////////////////////////////////////////////////////////////////////////////////////////
int _putf(double value, int width, int precision, char pad)
{
   int scale = 1;
   for (register int i = 0; i < precision; i++)
      scale *= 10;

   int integer = (int)value;
   int fractional = (value - integer) * scale;

   // Round up the fractional component to the required significant figures
   fractional *= 10;
   fractional += 5;
   fractional /= 10;

   // Special case, if we rounded up to the nearest whole number
   if (fractional >= scale)
   {
      fractional = 0;
      integer++;
   }

   if (pad == ' ' || pad == '0')
      width -= _puti(integer, width - 1 - precision, pad);
   else
      width -= _puti(integer, 0, pad);

   width -= _putc('.');
   width -= _putu(fractional, precision, '0');

   while (width-- > 0)
      _putc(' ');

   return 0; // This is a bit of a hack, we should return the number of characters
}
