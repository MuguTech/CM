/*
Original code by Lee Thomason (www.grinninglizard.com)

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any
damages arising from the use of this software.

Permission is granted to anyone to use this software for any
purpose, including commercial applications, and to alter it and
redistribute it freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this
software in a product, an acknowledgment in the product documentation
would be appreciated but is not required.

2. Altered source versions must be plainly marked as such, and
must not be misrepresented as being the original software.

3. This notice may not be removed or altered from any source
distribution.
*/

#include "simproxml.h"

#include <new>		// yes, this one new style header, is in the Android SDK.
#if defined(ANDROID_NDK) || defined(__BORLANDC__) || defined(__QNXNTO__)
#   include <stddef.h>
#   include <stdarg.h>
#else
#   include <cstddef>
#   include <cstdarg>
#endif




namespace simproxml
{

#if defined(_MSC_VER) && (_MSC_VER >= 1400 ) && (!defined WINCE)
    // Microsoft Visual Studio, version 2005 and higher. Not WinCE.
/*int _snprintf_s(
           char *buffer,
           size_t sizeOfBuffer,
           size_t count,
           const char *format [,
                  argument] ...
        );*/
static inline int32_t TIXML_SNPRINTF( char_t* buffer, size_t size, const char_t* format, ... )
{
    va_list va;
    va_start( va, format );
    const int32_t result = vsnprintf_s( buffer, size, _TRUNCATE, format, va );
    va_end( va );
    return result;
}

static inline int32_t TIXML_VSNPRINTF( char_t* buffer, size_t size, const char_t* format, va_list va )
{
    const int32_t result = vsnprintf_s( buffer, size, _TRUNCATE, format, va );
    return result;
}

#define TIXML_VSCPRINTF	_vscprintf
#define TIXML_SSCANF	sscanf_s
#elif defined _MSC_VER
    // Microsoft Visual Studio 2003 and earlier or WinCE
#define TIXML_SNPRINTF	_snprintf
#define TIXML_VSNPRINTF _vsnprintf
#define TIXML_SSCANF	sscanf
#if (_MSC_VER < 1400 ) && (!defined WINCE)
// Microsoft Visual Studio 2003 and not WinCE.
#define TIXML_VSCPRINTF   _vscprintf // VS2003's C runtime has this, but VC6 C runtime or WinCE SDK doesn't have.
#else
// Microsoft Visual Studio 2003 and earlier or WinCE.
static inline int32_t TIXML_VSCPRINTF( const char_t* format, va_list va )
{
    int32_t len = 512;
    for (;;)
    {
        len = len*2;
        char_t* str = new char_t[len]();
        const int32_t required = _vsnprintf(str, len, format, va);
        delete[] str;
        if ( required != -1 )
        {
            TIXMLASSERT( required >= 0 );
            len = required;
            break;
        }
    }
    TIXMLASSERT( len >= 0 );
    return len;
}
#endif
#else
// GCC version 3 and higher
//#warning( "Using sn* functions." )
#define TIXML_SNPRINTF	snprintf
#define TIXML_VSNPRINTF	vsnprintf
    static inline int32_t TIXML_VSCPRINTF( const char_t* const format, va_list va )
    {
        const int32_t len = vsnprintf( nullptr, 0, format, va );
        if(len != 0)
        {}

        TIXMLASSERT( len >= 0 );
        return len;
    }
#define TIXML_SSCANF   sscanf
#endif


//static const char LINE_FEED			= static_cast<char>(0x0a);			all line endings are normalized to LF
static const char LF                    = static_cast<char>(0x0a);
//static const char CARRIAGE_RETURN		= static_cast<char>(0x0d);			CR gets filtered out
static const char CR                    = static_cast<char>(0x0d);
//static const char SINGLE_QUOTE			= char('\'');
//static const char DOUBLE_QUOTE			= char('\"');

// Bunch of unicode info at:
/* http://www.unicode.org/faq/utf_bom.html */
//	ef bb bf (Microsoft "lead bytes") - designates UTF-8

static const uint8_t TIXML_UTF_LEAD_0 = 0xefU;
static const uint8_t TIXML_UTF_LEAD_1 = 0xbbU;
static const uint8_t TIXML_UTF_LEAD_2 = 0xbfU;

// A fixed element depth limit is problematic. There needs to be a
// limit to avoid a stack overflow. However, that limit varies per
// system, and the capacity of the stack. On the other hand, it's a trivial
// attack that can result from ill, malicious, or even correctly formed XML,
// so there needs to be a limit in place.
static const int32_t TINYXML2_MAX_ELEMENT_DEPTH = 100;

struct Entity
{
    const char_t* pattern;
    int32_t length;
    char_t value;
};

//static const int NUM_ENTITIES = 5;
static const Entity entities[5] = {
    { "quot", 4,	char('\"') },
    { "amp", 3,		'&'  },
    { "apos", 4,	char('\'') },
    { "lt",	2, 		'<'	 },
    { "gt",	2,		'>'	 }
};

void StrPair::Reset()
{
    const bool b = static_cast<uint32_t>(_flags) & static_cast<uint32_t>(NEEDS_DELETE);
    if ( b )
    {
        delete [] _start;
    }

    _flags = 0;
    _start = nullptr;
    _end = nullptr;
}

StrPair::~StrPair()
{
    Reset();
}

void StrPair::TransferTo( StrPair* other )
{
    if ( this != other )
    {
        // This in effect implements the assignment operator by "moving"
        // ownership (as in auto_ptr).
        TIXMLASSERT( other != nullptr);
        TIXMLASSERT( other->_flags == 0 );
        TIXMLASSERT( other->_start == nullptr );
        TIXMLASSERT( other->_end == nullptr);

        other->Reset();

        other->_flags = _flags;
        other->_start = _start;
        other->_end = _end;

        _flags = 0;
        _start = nullptr;
        _end = nullptr;
    }
}

void StrPair::SetStr( const char_t* str, int32_t flags )
{
    TIXMLASSERT( str );

    Reset();

    size_t len = std::string( str ).length();

    TIXMLASSERT( _start == nullptr);

    _start = new char_t[ len+1 ];
    if(_start == nullptr)
    {
        return;
    }

    (void)memcpy( _start, str, len+1 );
    _end = &_start[len];
    _flags = static_cast<uint32_t>(flags) | static_cast<uint32_t>(NEEDS_DELETE);
}

char_t* StrPair::ParseText( char_t* p, const char_t* endTag, int32_t strFlags, int32_t* curLineNumPtr )
{
    TIXMLASSERT( p );
    TIXMLASSERT( endTag && *endTag );
    TIXMLASSERT(curLineNumPtr);

    char_t* start = p;
    const char_t endChar = *endTag;
    const size_t length = std::string( endTag).length();

    // Inner loop of text parsing.
    while ( (*p) != '\0' )
    {
        if ( (*p == endChar) && (strncmp( p, endTag, length ) == 0) )
        {
            Set( start, p, strFlags );
            return &p[length];
        }
        else if (*p == '\n')
        {
            ++(*curLineNumPtr);
        }
        else
        {
            // comment: Nothing to do!
        }

        p = &p[1];
        TIXMLASSERT( p );
    }

    return nullptr;
}

char_t* StrPair::ParseName( char_t* p )
{
    if ( (p == nullptr) || ((*p) == '\0') )
    {
        return nullptr;
    }

    if ( !XMLUtil::IsNameStartChar( *p ) )
    {
        return nullptr;
    }

    char_t* const start = p;
    p = &p[1];

    while ( ((*p) != '\0') && XMLUtil::IsNameChar( *p ) )
    {
        p = &p[1];
    }

    Set( start, p, 0 );
    return p;
}


void StrPair::CollapseWhitespace()
{
    // Adjusting _start would cause undefined behavior on delete[]
    TIXMLASSERT( ( _flags & NEEDS_DELETE ) == 0 );
    // Trim leading space.
    _start = XMLUtil::SkipWhiteSpace( _start, nullptr);

    const bool b = *_start;
    if ( b )
    {
        char_t* p = _start;	// the read pointer
        char_t* q = _start;	// the write pointer

        while( (*p) != '\0' )
        {
            if ( XMLUtil::IsWhiteSpace( *p ))
            {
                p = XMLUtil::SkipWhiteSpace( p, nullptr);

                if ( (*p) == '\0' )
                {
                    break;    // don't write to q; this trims the trailing space.
                }

                *q = ' ';
                q = &q[1];
            }

            *q = *p;
            q = &q[1];
            p = &p[1];
        }

        *q = '\0';
    }
}


void XMLUtil::ConvertUTF32ToUTF8( uint64_t input, char_t* output, int32_t* length )
{
    const uint64_t BYTE_MASK = 0xBFU;
    const uint64_t BYTE_MARK = 0x80U;
    const uint64_t FIRST_BYTE_MARK[7] = { 0x00U, 0x00U, 0xC0U, 0xE0U, 0xF0U, 0xF8U, 0xFCU };

    if (input < 0x80U)
    {
        *length = 1;
    }
    else if ( input < 0x800U )
    {
        *length = 2;
    }
    else if ( input < 0x10000U )
    {
        *length = 3;
    }
    else if ( input < 0x200000U )
    {
        *length = 4;
    }
    else
    {
        *length = 0;    // This code won't convert this correctly anyway.
        return;
    }

    output = &output[*length];

    // Scary scary fall throughs are annotated with carefully designed comments
    // to suppress compiler warnings such as -Wimplicit-fallthrough in gcc
    switch (*length)
    {
    case 4:
    case 3:
    case 2:
        output = &output[-1];
        *output = static_cast<char_t>((static_cast<uint8_t>(input) | static_cast<uint8_t>(BYTE_MARK)) & static_cast<uint8_t>(BYTE_MASK));
        input >>= 6;
        //fall through
    case 1:
        output = &output[-1];
        *output = static_cast<char_t>(static_cast<uint8_t>(input) | static_cast<uint8_t>(FIRST_BYTE_MARK[*length]));
        break;
    default:
        TIXMLASSERT( false );
        break;
    }
}

const char_t* XMLUtil::GetCharacterRef( const char_t* p, char_t* value, int32_t* length )
{
    // Presume an entity, and pull it out.
    *length = 0;

    if ( (p[1] == '#') && (p[2] != 0) )
    {
        uint64_t ucs = 0;
        TIXMLASSERT( sizeof( ucs ) >= 4 );

        ptrdiff_t delta = 0;
        uint32_t mult = 1;
        static const char_t SEMICOLON = ';';

        if ( p[2] == 'x' )
        {
            // Hexadecimal.
            const char_t* q = &p[3];
            if ( (*q) == 0 )
            {
                return nullptr;
            }

            const size_t pos = std::string(q).find_first_of(SEMICOLON);
            q = (pos != std::string::npos) ? &q[pos] : nullptr;

            if ( q == nullptr )
            {
                return nullptr;
            }
            TIXMLASSERT( *q == SEMICOLON );

            delta = q-p;
            q = &q[-1];

            while ( *q != 'x' )
            {
                uint32_t digit = 0;

                if ( (*q >= '0') && (*q <= '9') )
                {
                    digit = static_cast<uint32_t>(*q) - static_cast<uint32_t>('0');
                }
                else if ( (*q >= 'a') && (*q <= 'f') )
                {
                    digit = static_cast<uint32_t>(*q) - static_cast<uint32_t>('a') + 10;
                }
                else if ( (*q >= 'A') && (*q <= 'F') )
                {
                    digit = static_cast<uint32_t>(*q) - static_cast<uint32_t>('A') + 10;
                }
                else
                {
                    return nullptr;
                }

                TIXMLASSERT( digit < 16 );
                TIXMLASSERT( digit == 0 || (mult <= (UINT_MAX / digit)) );

                const uint32_t digitScaled = mult * digit;
                TIXMLASSERT( ucs <= (ULONG_MAX - digitScaled) );

                ucs += digitScaled;
                TIXMLASSERT( mult <= (UINT_MAX / 16) );

                mult *= 16;
                q = &q[-1];
            }
        }
        else
        {
            // Decimal.
            const char_t* q = &p[2];
            if ( (*q) == 0 )
            {
                return nullptr;
            }

            const size_t pos = std::string(q).find_first_of(SEMICOLON);
            q = (pos != std::string::npos) ? &q[pos] : nullptr;


            if ( q == nullptr )
            {
                return nullptr;
            }
            TIXMLASSERT( *q == SEMICOLON );

            delta = q-p;
            q = &q[-1];

            while ( *q != '#' )
            {
                if ( (*q >= '0') && (*q <= '9') )
                {
                    const uint32_t digit = static_cast<uint32_t>(*q) - static_cast<uint32_t>('0');
                    TIXMLASSERT( digit < 10 );
                    TIXMLASSERT( (digit == 0) || (mult <= UINT_MAX / digit) );

                    const uint32_t digitScaled = mult * digit;
                    TIXMLASSERT( ucs <= (ULONG_MAX - digitScaled) );

                    ucs += digitScaled;
                }
                else
                {
                    return nullptr;
                }

                TIXMLASSERT( mult <= (UINT_MAX / 10) );
                mult *= 10;
                q = &q[-1];
            }
        }

        // convert the UCS to UTF-8
        ConvertUTF32ToUTF8( ucs, value, length );
        return &p[delta + 1];
    }

    return &p[1];
}

const char_t* StrPair::GetStr()
{
    TIXMLASSERT( _start );
    TIXMLASSERT( _end );

    if ( (static_cast<uint32_t>(_flags) & static_cast<uint32_t>(NEEDS_FLUSH)) != 0 )
    {
        *_end = '\0';
        _flags = static_cast<uint32_t>(_flags) ^ static_cast<uint32_t>(NEEDS_FLUSH);

        const bool b2 = _flags;
        if ( b2 )
        {
            const char_t* p = _start;	// the read pointer
            char_t* q = _start;	// the write pointer

            const uint32_t n = NEEDS_NEWLINE_NORMALIZATION;
            const uint32_t nn = NEEDS_ENTITY_PROCESSING;

            while( p < _end )
            {
                if ( ((static_cast<uint32_t>(_flags) & n) != 0) && ((*p) == CR) )
                {
                    // CR-LF pair becomes LF
                    // CR alone becomes LF
                    // LF-CR becomes LF
                    if ( p[1] == LF )
                    {
                        p = &p[2];
                    }
                    else
                    {
                        p = &p[1];
                    }

                    *q = LF;
                    q = &q[1];
                }
                else if ( ((static_cast<uint32_t>(_flags) & n)!= 0) && (*p == LF) )
                {
                    if ( p[1] == CR )
                    {
                        p = &p[2];
                    }
                    else
                    {
                        p = &p[1];
                    }

                    *q = LF;
                    q = &q[1];
                }
                else if ( ((static_cast<uint32_t>(_flags) & nn) != 0) && ((*p) == '&') )
                {
                    // Entities handled by tinyXML2:
                    // - special entities in the entity table [in/out]
                    // - numeric character reference [in]
                    //   &#20013; or &#x4e2d;

                    if ( p[1] == '#' )
                    {
                        char_t buf[10] = { 0 };
                        int32_t len = 0;
                        const char* adjusted = XMLUtil::GetCharacterRef( p, &buf[0], &len );

                        if ( adjusted == nullptr )
                        {
                            *q = *p;
                            p = &p[1];
                            q = &q[1];
                        }
                        else
                        {
                            TIXMLASSERT( (0 <= len) && (len <= sizeof(buf)) );
                            TIXMLASSERT( (q + len) <= adjusted );

                            p = adjusted;
                            (void)memcpy( q, buf, len );
                            q = &q[len];
                        }
                    }
                    else
                    {
                        bool entityFound = false;
                        for( const Entity& entity : entities )
                        {
                            if (
                                 (strncmp( &p[1], entity.pattern, entity.length ) == 0)
                                 && (p[entity.length + 1    ] == ';')
                               )
                            {
                                // Found an entity - convert.
                                *q = entity.value;
                                q = &q[1];
                                p = &p[entity.length + 2];
                                entityFound = true;
                                break;
                            }
                        }

                        if ( !entityFound )
                        {
                            // fixme: treat as error?
                            p = &p[1];
                            q = &q[1];
                        }
                    }
                }
                else
                {
                    *q = *p;
                    p = &p[1];
                    q = &q[1];
                }
            }

            *q = '\0';
            static_cast<void>(q);
        }

        // The loop below has plenty going on, and this
        // is a less useful mode. Break it out.
        const bool b = ((static_cast<uint32_t>(_flags) & static_cast<uint32_t>(NEEDS_WHITESPACE_COLLAPSING)) != 0);
        if ( b )
        {
            CollapseWhitespace();
        }

        _flags = static_cast<uint32_t>(_flags) & static_cast<uint32_t>(NEEDS_DELETE);
    }

    TIXMLASSERT( _start );
    return _start;
}



// --------- XMLUtil -----------

const char_t* XMLUtil::writeBoolTrue  = "true";
const char_t* XMLUtil::writeBoolFalse = "false";

void XMLUtil::SetBoolSerialization(const char_t* writeTrue, const char_t* writeFalse)
{
    static const char_t* defTrue  = "true";
    static const char_t* defFalse = "false";

    writeBoolTrue = (writeTrue != nullptr) ? writeTrue : defTrue;
    writeBoolFalse = (writeFalse != nullptr) ? writeFalse : defFalse;
}

const char_t* XMLUtil::ReadBOM( const char_t* p, bool* bom )
{
    TIXMLASSERT( p );
    TIXMLASSERT( bom );

    *bom = false;
    const uint8_t* pu = reinterpret_cast<const uint8_t*>(p);
    // Check for BOM:
    if (
            (pu[0] == TIXML_UTF_LEAD_0)
            && (pu[1] == TIXML_UTF_LEAD_1)
            && (pu[2] == TIXML_UTF_LEAD_2)
       )
    {
        *bom = true;
        p = &p[3];
    }

    TIXMLASSERT( p );
    return p;
}

void XMLUtil::ToStr( int32_t v, char_t* const buffer, int32_t bufferSize )
{
    const int32_t i = TIXML_SNPRINTF( buffer, bufferSize, "%d", v );
    if ( i < 0 )
    {}
}

void XMLUtil::ToStr( uint32_t v, char_t* const buffer, int32_t bufferSize )
{
    const int32_t i = TIXML_SNPRINTF( buffer, bufferSize, "%u", v );
    if ( i < 0 )
    {}
}

void XMLUtil::ToStr( bool v, char_t* const buffer, int32_t bufferSize )
{
    const int32_t i = TIXML_SNPRINTF( buffer, bufferSize, "%s", v ? writeBoolTrue : writeBoolFalse);
    if ( i < 0 )
    {}
}

/*
        ToStr() of a number is a very tricky topic.
        https://github.com/leethomason/tinyxml2/issues/106
*/
void XMLUtil::ToStr( float32_t v, char_t* const buffer, int32_t bufferSize )
{
    const int32_t i = TIXML_SNPRINTF( buffer, bufferSize, "%.8g", v );
    if ( i < 0 )
    {}
}

void XMLUtil::ToStr( float64_t v, char_t* const buffer, int32_t bufferSize )
{
    const int32_t i = TIXML_SNPRINTF( buffer, bufferSize, "%.17g", v );
    if ( i < 0 )
    {}
}

void XMLUtil::ToStr( int64_t v, char_t* const buffer, int32_t bufferSize )
{
    // horrible syntax trick to make the compiler happy about %lld
    const int32_t i = TIXML_SNPRINTF(buffer, bufferSize, "%ld", static_cast<int64_t>(v));
    if ( i < 0 )
    {}
}

void XMLUtil::ToStr( uint64_t v, char_t* const buffer, int32_t bufferSize )
{
    // horrible syntax trick to make the compiler happy about %llu
    const int32_t i = TIXML_SNPRINTF(buffer, bufferSize, "%lu", static_cast<int64_t>(v));
    if ( i < 0 )
    {}
}

bool XMLUtil::ToInt( const char_t* const str, int32_t* value )
{
    const bool b = ( TIXML_SSCANF( str, "%d", value ) == 1 ) ? true : false;
    return b;
}

bool XMLUtil::ToUnsigned( const char_t* const str, uint32_t *value )
{
    const bool b = ( TIXML_SSCANF( str, "%u", value ) == 1 ) ? true : false;
    return b;
}

bool XMLUtil::ToBool( const char_t* const str, bool* const value )
{
    int32_t ival = 0;
    const bool b = ToInt( str, &ival );
    if ( b )
    {
        *value = (ival==0) ? false : true;
        return true;
    }

    static const char_t* const TRUE_VALS[] = { "true", "True", "TRUE", nullptr };
    static const char_t* const FALSE_VALS[] = { "false", "False", "FALSE", nullptr };

    for (int32_t i = 0; TRUE_VALS[i] != nullptr; ++i)
    {
        if ( StringEqual(str, TRUE_VALS[i]) )
        {
            *value = true;
            return true;
        }
    }

    for (int32_t i = 0; FALSE_VALS[i] != nullptr; ++i)
    {
        if (StringEqual(str, FALSE_VALS[i]))
        {
            *value = false;
            return true;
        }
    }

    return false;
}

bool XMLUtil::ToFloat( const char_t* str, float32_t* value )
{
    const bool b = ( TIXML_SSCANF( str, "%f", value ) == 1 ) ? true : false;
    return b;
}

bool XMLUtil::ToDouble( const char_t* str, float64_t* value )
{
    const bool b = ( TIXML_SSCANF( str, "%lf", value ) == 1 ) ? true : false;
    return b;
}

bool XMLUtil::ToInt64(const char_t* str, int64_t* value)
{
    int64_t v = 0;	// horrible syntax trick to make the compiler happy about %lld
    if (TIXML_SSCANF(str, "%ld", &v) == 1)
    {
        *value = static_cast<int64_t>(v);
        return true;
    }

    return false;
}

bool XMLUtil::ToUnsigned64(const char_t* str, uint64_t* value)
{
    uint128_t v = 0;	// horrible syntax trick to make the compiler happy about %llu
    if(TIXML_SSCANF(str, "%llu", &v) == 1)
    {
        *value = static_cast<uint64_t>(v);
        return true;
    }

    return false;
}


char_t* XMLDocument::Identify( char_t* p, XMLNode** node )
{
    TIXMLASSERT( node );
    TIXMLASSERT( p );

    char_t* const start = p;
    int32_t const startLine = _parseCurLineNum;

    p = XMLUtil::SkipWhiteSpace( p, &_parseCurLineNum );
    if( (*p) == '\0' )
    {
        *node = nullptr;
        TIXMLASSERT( p );
        return p;
    }

    // These strings define the matching patterns:
    static const char_t* xmlHeader		= { "<?" };
    static const char_t* commentHeader	= { "<!--" };
    static const char_t* cdataHeader	= { "<![CDATA[" };
    static const char_t* dtdHeader		= { "<!" };
    static const char_t* elementHeader	= { "<" };	// and a header for everything else; check last.

    static const int32_t xmlHeaderLen		= 2;
    static const int32_t commentHeaderLen	= 4;
    static const int32_t cdataHeaderLen		= 9;
    static const int32_t dtdHeaderLen		= 2;
    static const int32_t elementHeaderLen	= 1;

    TIXMLASSERT( sizeof( XMLComment ) == sizeof( XMLUnknown ) );		// use same memory pool
    TIXMLASSERT( sizeof( XMLComment ) == sizeof( XMLDeclaration ) );	// use same memory pool

    XMLNode* returnNode = nullptr;
    if ( XMLUtil::StringEqual( p, xmlHeader, xmlHeaderLen ) )
    {
        returnNode = CreateUnlinkedNode<XMLDeclaration>( _commentPool );
        returnNode->_parseLineNum = _parseCurLineNum;
        p = &p[xmlHeaderLen];
    }
    else if ( XMLUtil::StringEqual( p, commentHeader, commentHeaderLen ) )
    {
        returnNode = CreateUnlinkedNode<XMLComment>( _commentPool );
        returnNode->_parseLineNum = _parseCurLineNum;
        p = &p[commentHeaderLen];
    }
    else if ( XMLUtil::StringEqual( p, cdataHeader, cdataHeaderLen ) )
    {
        auto* text = CreateUnlinkedNode<XMLText>( _textPool );
        returnNode = text;
        returnNode->_parseLineNum = _parseCurLineNum;
        p = &p[cdataHeaderLen];
        text->SetCData( true );
    }
    else if ( XMLUtil::StringEqual( p, dtdHeader, dtdHeaderLen ) )
    {
        returnNode = CreateUnlinkedNode<XMLUnknown>( _commentPool );
        returnNode->_parseLineNum = _parseCurLineNum;
        p = &p[dtdHeaderLen];
    }
    else if ( XMLUtil::StringEqual( p, elementHeader, elementHeaderLen ) )
    {
        returnNode =  CreateUnlinkedNode<XMLElement>( _elementPool );
        returnNode->_parseLineNum = _parseCurLineNum;
        p = &p[elementHeaderLen];
    }
    else
    {
        returnNode = CreateUnlinkedNode<XMLText>( _textPool );
        returnNode->_parseLineNum = _parseCurLineNum; // Report line of first non-whitespace character
        p = start;	// Back it up, all the text counts.
        _parseCurLineNum = startLine;
    }

    TIXMLASSERT( returnNode );
    TIXMLASSERT( p );
    *node = returnNode;
    return p;
}

bool XMLDocument::Accept( XMLVisitor* visitor ) const
{
    TIXMLASSERT( visitor );
    if ( visitor->VisitEnter( *this ) )
    {
        for ( const XMLNode* node=FirstChild(); node != nullptr; node=node->NextSibling() )
        {
            const bool b = !node->Accept( visitor );
            if ( b )
            {
                break;
            }
        }
    }

    return visitor->VisitExit( *this );
}


// --------- XMLNode -----------

XMLNode::XMLNode( XMLDocument* const doc ) :
    _document( doc ),
    _parent(nullptr),
    _value(),
    _parseLineNum( 0 ),
    _firstChild(nullptr), _lastChild(nullptr),
    _prev(nullptr), _next(nullptr),
    _userData(nullptr),
    _memPool(nullptr)
{
}

void XMLNode::DeleteNode( XMLNode* node )
{
    if ( node != nullptr)
    {
        TIXMLASSERT(node->_document);
        if (node->ToDocument() == nullptr)
        {
            node->_document->MarkInUse(node);
        }

        MemPool* pool = node->_memPool;
        node->~XMLNode();
        pool->Free( node );
    }
}

void XMLNode::DeleteChild( XMLNode* node )
{
    TIXMLASSERT( node );
    TIXMLASSERT( node->_document == _document );
    TIXMLASSERT( node->_parent == this );
    (Unlink)( node );

    TIXMLASSERT(node->_prev == nullptr);
    TIXMLASSERT(node->_next == nullptr);
    TIXMLASSERT(node->_parent == nullptr);
    (DeleteNode)( node );
}

void XMLNode::DeleteChildren()
{
    while( _firstChild != nullptr )
    {
        TIXMLASSERT( _lastChild );
        DeleteChild( _firstChild );
    }

    _firstChild = nullptr;
    _lastChild = nullptr;
}

XMLNode::~XMLNode()
{
    DeleteChildren();
    if ( _parent != nullptr )
    {
        _parent->Unlink( this );
    }
}

const char_t* XMLNode::Value() const
{
    // Edge case: XMLDocuments don't have a Value. Return null.
    const char_t *pCh = ( this->ToDocument() != nullptr ) ? nullptr : _value.GetStr();
    return pCh;
}

void XMLNode::SetValue( const char_t* str, bool staticMem )
{
    if ( staticMem )
    {
        _value.SetInternedStr( str );
    }
    else
    {
        _value.SetStr( str );
    }
}

void XMLNode::Unlink( XMLNode* child )
{
    TIXMLASSERT( child );
    TIXMLASSERT( child->_document == _document );
    TIXMLASSERT( child->_parent == this );

    if ( child == _firstChild )
    {
        _firstChild = _firstChild->_next;
    }
    if ( child == _lastChild )
    {
        _lastChild = _lastChild->_prev;
    }

    if ( child->_prev != nullptr )
    {
        child->_prev->_next = child->_next;
    }
    if ( child->_next != nullptr )
    {
        child->_next->_prev = child->_prev;
    }

    child->_next = nullptr;
    child->_prev = nullptr;
    child->_parent = nullptr;
}

void XMLNode::InsertChildPreamble( XMLNode* insertThis ) const
{
    TIXMLASSERT( insertThis );
    TIXMLASSERT( insertThis->_document == _document );

    if (insertThis->_parent != nullptr)
    {
        insertThis->_parent->Unlink( insertThis );
    }
    else
    {
        insertThis->_document->MarkInUse(insertThis);
        insertThis->_memPool->SetTracked();
    }
}

XMLNode* XMLNode::InsertEndChild( XMLNode* addThis )
{
    TIXMLASSERT( addThis );
    if ( addThis->_document != _document )
    {
        TIXMLASSERT( false );
        return nullptr;
    }

    InsertChildPreamble( addThis );

    if ( _lastChild != nullptr )
    {
        TIXMLASSERT( _firstChild );
        TIXMLASSERT( _lastChild->_next == nullptr);
        _lastChild->_next = addThis;
        addThis->_prev = _lastChild;
        _lastChild = addThis;

        addThis->_next = nullptr;
    }
    else
    {
        TIXMLASSERT( _firstChild == nullptr);
        _firstChild = addThis; _lastChild = addThis;

        addThis->_prev = nullptr;
        addThis->_next = nullptr;
    }

    addThis->_parent = this;
    return addThis;
}

XMLNode* XMLNode::DeepClone(XMLDocument* target) const
{
    XMLNode* clone = this->ShallowClone(target);
    if (clone == nullptr)
    {
        return nullptr;
    }

    for ( const XMLNode* child = this->FirstChild(); child != nullptr; child = child->NextSibling() )
    {
        XMLNode* childClone = child->DeepClone(target);
        TIXMLASSERT(childClone);
        (void)clone->InsertEndChild(childClone);
    }

    return clone;
}

XMLNode* XMLNode::InsertFirstChild( XMLNode* addThis )
{
    TIXMLASSERT( addThis );
    if ( addThis->_document != _document )
    {
        TIXMLASSERT( false );
        return nullptr;
    }

    InsertChildPreamble( addThis );

    if ( _firstChild != nullptr )
    {
        TIXMLASSERT( _lastChild );
        TIXMLASSERT( _firstChild->_prev == nullptr);

        _firstChild->_prev = addThis;
        addThis->_next = _firstChild;
        _firstChild = addThis;

        addThis->_prev = nullptr;
    }
    else
    {
        TIXMLASSERT( _lastChild == nullptr);
        _firstChild = addThis;
        _lastChild = addThis;

        addThis->_prev = nullptr;
        addThis->_next = nullptr;
    }

    addThis->_parent = this;
    return addThis;
}

XMLNode* XMLNode::InsertAfterChild( XMLNode* afterThis, XMLNode* addThis )
{
    TIXMLASSERT( addThis );
    if ( addThis->_document != _document )
    {
        TIXMLASSERT( false );
        return nullptr;
    }

    TIXMLASSERT( afterThis );

    if ( afterThis->_parent != this )
    {
        TIXMLASSERT( false );
        return nullptr;
    }

    if ( afterThis == addThis )
    {
        // Current state: BeforeThis -> AddThis -> OneAfterAddThis
        // Now AddThis must disappear from it's location and then
        // reappear between BeforeThis and OneAfterAddThis.
        // So just leave it where it is.
        return addThis;
    }

    if ( afterThis->_next == nullptr)
    {
        // The last node or the only node.
        return InsertEndChild( addThis );
    }

    InsertChildPreamble( addThis );
    addThis->_prev = afterThis;
    addThis->_next = afterThis->_next;
    afterThis->_next->_prev = addThis;
    afterThis->_next = addThis;
    addThis->_parent = this;

    return addThis;
}

const XMLElement* XMLNode::ToElementWithName( const char_t* const name ) const
{
    const XMLElement* element = this->ToElement();
    if ( element == nullptr)
    {
        return nullptr;
    }

    if ( name == nullptr)
    {
        return element;
    }

    if ( XMLUtil::StringEqual( element->Name(), name ) )
    {
        return element;
    }

    return nullptr;
}

const XMLElement* XMLNode::FirstChildElement( const char_t* const name ) const
{
    const XMLElement* element = nullptr;
    for( const XMLNode* node = _firstChild; node != nullptr; node = node->_next )
    {
        element = node->ToElementWithName( name );
        if ( element != nullptr )
        {
            break;
        }
    }

    return element;
}

const XMLElement* XMLNode::LastChildElement( const char_t* const name ) const
{
    const XMLElement *element = nullptr;
    for( const XMLNode* node = _lastChild; node != nullptr; node = node->_prev )
    {
        element = node->ToElementWithName( name );
        if ( element != nullptr )
        {
            break;
        }
    }

    return element;
}

const XMLElement* XMLNode::NextSiblingElement( const char_t* const name ) const
{
    const XMLElement *element = nullptr;
    for( const XMLNode* node = _next; node != nullptr; node = node->_next )
    {
        element = node->ToElementWithName( name );
        if ( element != nullptr )
        {
            break;
        }
    }

    return element;
}

const XMLElement* XMLNode::PreviousSiblingElement( const char_t* const name ) const
{
    const XMLElement *element = nullptr;
    for( const XMLNode* node = _prev; node != nullptr; node = node->_prev )
    {
        element = node->ToElementWithName( name );
        if ( element != nullptr )
        {
            break;
        }
    }

    return element;
}

char_t* XMLNode::ParseDeep( char_t* p, StrPair* const parentEndTag, int32_t* const curLineNumPtr )
{
    // This is a recursive method, but thinking about it "at the current level"
    // it is a pretty simple flat list:
    //		<foo/>
    //		<!-- comment -->
    //
    // With a special case:
    //		<foo>
    //		</foo>
    //		<!-- comment -->
    //
    // Where the closing element (/foo) *must* be the next thing after the opening
    // element, and the names must match. BUT the tricky bit is that the closing
    // element will be read by the child.
    //
    // 'endTag' is the end tag for this node, it is returned by a call to a child.
    // 'parentEnd' is the end tag for the parent, which is filled in and returned.

    const XMLDocument::DepthTracker tracker(_document);

    bool b = _document->Error();
    if ( b )
    {
        return nullptr;
    }

    while( (p != nullptr) && ((*p) != '\0') )
    {
        XMLNode* node = nullptr;

        p = _document->Identify( p, &node );
        TIXMLASSERT( p );

        if ( node == nullptr)
        {
            break;
        }

        const int32_t initialLineNum = node->_parseLineNum;

        StrPair endTag;
        p = node->ParseDeep( p, &endTag, curLineNumPtr );
        if ( p == nullptr )
        {
            DeleteNode( node );
            b = !_document->Error();
            if ( b )
            {
                _document->SetError( XML_ERROR_PARSING, initialLineNum, nullptr);
            }

            break;
        }

        const XMLDeclaration* const decl = node->ToDeclaration();
        if ( (decl != nullptr) )
        {
            // Declarations are only allowed at document level
            //
            // Multiple declarations are allowed but all declarations
            // must occur before anything else.
            //
            // Optimized due to a security test case. If the first node is
            // a declaration, and the last node is a declaration, then only
            // declarations have so far been added.
            bool wellLocated = false;

            if ( (ToDocument() != nullptr) )
            {
                if ( (FirstChild() != nullptr) )
                {
                    wellLocated = (
                                     (FirstChild() != nullptr) &&
                                     ( FirstChild()->ToDeclaration() != nullptr ) &&
                                     (LastChild() != nullptr) &&
                                     (LastChild()->ToDeclaration() != nullptr )
                                  );
                }
                else
                {
                    wellLocated = true;
                }
            }
            else
            {
                // comment: Nothing to do!
            }

            if ( !wellLocated )
            {
                _document->SetError( XML_ERROR_PARSING_DECLARATION, initialLineNum, "XMLDeclaration value=%s", decl->Value());
                DeleteNode( node );
                break;
            }
            else
            {
                // comment: Nothing to do!
            }
        }
        else
        {
            // comment: Nothing to do!
        }

        XMLElement* ele = node->ToElement();
        if ( ele != nullptr )
        {
            // We read the end tag. Return it to the parent.
            if ( ele->ClosingType() == XMLElement::CLOSING )
            {
                if ( parentEndTag != nullptr )
                {
                    ele->_value.TransferTo( parentEndTag );
                }

                node->_memPool->SetTracked();   // created and then immediately deleted.
                DeleteNode( node );
                return p;
            }
            else
            {
                // comment: Nothing to do!
            }

            // Handle an end tag returned to this level.
            // And handle a bunch of annoying errors.
            bool mismatch = false;
            if ( endTag.Empty() )
            {
                if ( ele->ClosingType() == XMLElement::OPEN )
                {
                    mismatch = true;
                }
                else
                {
                    // comment: Nothing to do!
                }
            }
            else
            {
                if ( ele->ClosingType() != XMLElement::OPEN )
                {
                    mismatch = true;
                }
                else if ( !XMLUtil::StringEqual( endTag.GetStr(), ele->Name() ) )
                {
                    mismatch = true;
                }
                else
                {
                    // comment: Nothing to do!
                }
            }

            if ( mismatch )
            {
                _document->SetError( XML_ERROR_MISMATCHED_ELEMENT, initialLineNum, "XMLElement name=%s", ele->Name());
                DeleteNode( node );
                break;
            }
            else
            {
                // comment: Nothing to do!
            }
        }
        else
        {
            // comment: Nothing to do!
        }

        (void)InsertEndChild( node );
    }

    return nullptr;
}


// --------- XMLText ----------
char_t* XMLText::ParseDeep( char_t* p, StrPair*, int32_t* curLineNumPtr )
{
    if ( this->CData() )
    {
        p = _value.ParseText( p, "]]>", StrPair::NEEDS_NEWLINE_NORMALIZATION, curLineNumPtr );
        if ( p == nullptr )
        {
            _document->SetError( XML_ERROR_PARSING_CDATA, GetParseLineNum(), nullptr);
        }

        return p;
    }
    else
    {
        int32_t flags = _document->ProcessEntities() ? StrPair::TEXT_ELEMENT : StrPair::TEXT_ELEMENT_LEAVE_ENTITIES;
        if ( _document->WhitespaceMode() == COLLAPSE_WHITESPACE )
        {
            flags = static_cast<int32_t>(static_cast<uint32_t>(flags) | static_cast<uint32_t>(StrPair::NEEDS_WHITESPACE_COLLAPSING));
        }
        else
        {
            // comment: Nothing to do!
        }

        p = _value.ParseText( p, "<", flags, curLineNumPtr );
        if ( (p != nullptr) && ((*p) != '\0') )
        {
            return &p[-1];
        }
        else
        {
            // comment: Nothing to do!
        }

        if ( p == nullptr )
        {
            _document->SetError( XML_ERROR_PARSING_TEXT, GetParseLineNum(), nullptr);
        }
        else
        {
            // comment: Nothing to do!
        }
    }

    return nullptr;
}

XMLNode* XMLText::ShallowClone( XMLDocument* doc ) const
{
    if ( doc == nullptr )
    {
        doc = _document;
    }

    XMLText* text = doc->NewText( Value() );	// fixme: this will always allocate memory. Intern?
    text->SetCData( this->CData() );
    return text;
}

bool XMLText::ShallowEqual( const XMLNode* compare ) const
{
    TIXMLASSERT( compare );
    const XMLText* text = compare->ToText();
    return ( (text != nullptr) && XMLUtil::StringEqual( text->Value(), Value() ) );
}

bool XMLText::Accept( XMLVisitor* visitor ) const
{
    TIXMLASSERT( visitor );
    return visitor->Visit( *this );
}


// --------- XMLComment ----------

XMLComment::XMLComment( XMLDocument* doc ) : XMLNode( doc )
{
}


XMLComment::~XMLComment()
{
}

char_t* XMLComment::ParseDeep( char_t* p, StrPair*, int32_t* curLineNumPtr )
{
    // Comment parses as text.
    p = _value.ParseText( p, "-->", StrPair::COMMENT, curLineNumPtr );
    if ( p == nullptr)
    {
        _document->SetError( XML_ERROR_PARSING_COMMENT, GetParseLineNum(), nullptr);
    }

    return p;
}

XMLNode* XMLComment::ShallowClone( XMLDocument* doc ) const
{
    if ( doc == nullptr )
    {
        doc = _document;
    }

    XMLComment* const comment = doc->NewComment( Value() );	// fixme: this will always allocate memory. Intern?
    return comment;
}

bool XMLComment::ShallowEqual( const XMLNode* compare ) const
{
    TIXMLASSERT( compare );
    const XMLComment* comment = compare->ToComment();
    return ( (comment != nullptr) && XMLUtil::StringEqual( comment->Value(), Value() ));
}

bool XMLComment::Accept( XMLVisitor* visitor ) const
{
    TIXMLASSERT( visitor );
    return visitor->Visit( *this );
}


// --------- XMLDeclaration ----------

XMLDeclaration::XMLDeclaration( XMLDocument* doc ) : XMLNode( doc )
{
}


XMLDeclaration::~XMLDeclaration()
{
}

char_t* XMLDeclaration::ParseDeep( char_t* p, StrPair*, int32_t* curLineNumPtr )
{
    // Declaration parses as text.
    p = _value.ParseText( p, "?>", StrPair::NEEDS_NEWLINE_NORMALIZATION, curLineNumPtr );
    if ( p == nullptr)
    {
        _document->SetError( XML_ERROR_PARSING_DECLARATION, GetParseLineNum(), nullptr);
    }

    return p;
}

XMLNode* XMLDeclaration::ShallowClone( XMLDocument* doc ) const
{
    if ( doc == nullptr )
    {
        doc = _document;
    }

    XMLDeclaration* const dec = doc->NewDeclaration( Value() );	// fixme: this will always allocate memory. Intern?
    return dec;
}

bool XMLDeclaration::ShallowEqual( const XMLNode* compare ) const
{
    TIXMLASSERT( compare );
    const XMLDeclaration* declaration = compare->ToDeclaration();
    return ( (declaration != nullptr) && XMLUtil::StringEqual( declaration->Value(), Value() ));
}

bool XMLDeclaration::Accept( XMLVisitor* visitor ) const
{
    TIXMLASSERT( visitor );
    return visitor->Visit( *this );
}


// --------- XMLUnknown ----------

XMLUnknown::XMLUnknown( XMLDocument* doc ) : XMLNode( doc )
{
}

XMLUnknown::~XMLUnknown()
{
}

char_t* XMLUnknown::ParseDeep( char_t* p, StrPair*, int32_t* curLineNumPtr )
{
    // Unknown parses as text.
    p = _value.ParseText( p, ">", StrPair::NEEDS_NEWLINE_NORMALIZATION, curLineNumPtr );
    if ( p == nullptr )
    {
        _document->SetError( XML_ERROR_PARSING_UNKNOWN, GetParseLineNum(), nullptr );
    }

    return p;
}

XMLNode* XMLUnknown::ShallowClone( XMLDocument* doc ) const
{
    if ( doc == nullptr )
    {
        doc = _document;
    }

    XMLUnknown* const text = doc->NewUnknown( Value() );	// fixme: this will always allocate memory. Intern?

    return text;
}

bool XMLUnknown::ShallowEqual( const XMLNode* compare ) const
{
    TIXMLASSERT( compare );
    const XMLUnknown* unknown = compare->ToUnknown();
    return ( (unknown != nullptr) && XMLUtil::StringEqual( unknown->Value(), Value() ));
}

bool XMLUnknown::Accept( XMLVisitor* visitor ) const
{
    TIXMLASSERT( visitor );
    return visitor->Visit( *this );
}


// --------- XMLAttribute ----------

const char_t* XMLAttribute::Name() const
{
    return _name.GetStr();
}

const char_t* XMLAttribute::Value() const
{
    return _value.GetStr();
}

char_t* XMLAttribute::ParseDeep( char_t* p, bool processEntities, int32_t* curLineNumPtr )
{
    // Parse using the name rules: bug fix, was using ParseText before
    p = _name.ParseName( p );
    if ( (p == nullptr) || ((*p) == '\0') )
    {
        return nullptr;
    }

    // Skip white space before =
    p = XMLUtil::SkipWhiteSpace( p, curLineNumPtr );
    if ( (*p) != '=' )
    {
        return nullptr;
    }

    p = &p[1]; // move up to opening quote
    p = XMLUtil::SkipWhiteSpace( p, curLineNumPtr );
    if ( ((*p) != '\"') && ((*p) != '\'') )
    {
        return nullptr;
    }

    const char_t endTag[2] = { *p, 0 };
    p = &p[1]; // move past opening quote

    p = _value.ParseText( p, &endTag[0], processEntities ? StrPair::ATTRIBUTE_VALUE : StrPair::ATTRIBUTE_VALUE_LEAVE_ENTITIES, curLineNumPtr );

    return p;
}

void XMLAttribute::SetName( const char_t* n )
{
    _name.SetStr( n );
}

XMLError XMLAttribute::QueryIntValue( int32_t* value ) const
{
    const XMLError error = ( XMLUtil::ToInt( Value(), value )) ? XML_SUCCESS : XML_WRONG_ATTRIBUTE_TYPE;
    return error;
}

XMLError XMLAttribute::QueryUnsignedValue( uint32_t* value ) const
{
    const XMLError error = ( XMLUtil::ToUnsigned( Value(), value )) ? XML_SUCCESS : XML_WRONG_ATTRIBUTE_TYPE;
    return error;
}

XMLError XMLAttribute::QueryInt64Value(int64_t* value) const
{
    const XMLError error = (XMLUtil::ToInt64(Value(), value)) ? XML_SUCCESS : XML_WRONG_ATTRIBUTE_TYPE;
    return error;
}

XMLError XMLAttribute::QueryUnsigned64Value(uint64_t* value) const
{
    const XMLError error = (XMLUtil::ToUnsigned64(Value(), value)) ? XML_SUCCESS : XML_WRONG_ATTRIBUTE_TYPE;
    return error;
}

XMLError XMLAttribute::QueryBoolValue( bool* value ) const
{
    const XMLError error =  ( XMLUtil::ToBool( Value(), value )) ? XML_SUCCESS : XML_WRONG_ATTRIBUTE_TYPE;
    return error;
}

XMLError XMLAttribute::QueryFloatValue( float32_t* value ) const
{
    const XMLError error =  XMLUtil::ToFloat( Value(), value ) ? XML_SUCCESS : XML_WRONG_ATTRIBUTE_TYPE;
    return error;
}

XMLError XMLAttribute::QueryDoubleValue( float64_t* value ) const
{
    const XMLError error = ( XMLUtil::ToDouble( Value(), value )) ? XML_SUCCESS : XML_WRONG_ATTRIBUTE_TYPE;
    return error;
}

void XMLAttribute::SetAttribute( const char_t* v )
{
    _value.SetStr( v );
}

void XMLAttribute::SetAttribute( int32_t v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    _value.SetStr( buf );
}

void XMLAttribute::SetAttribute( uint32_t v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    _value.SetStr( buf );
}

void XMLAttribute::SetAttribute(int64_t v)
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr(v, &buf[0], BUF_SIZE);
    _value.SetStr(buf);
}

void XMLAttribute::SetAttribute(uint64_t v)
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr(v, &buf[0], BUF_SIZE);
    _value.SetStr(buf);
}

void XMLAttribute::SetAttribute( bool v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    _value.SetStr( buf );
}

void XMLAttribute::SetAttribute( float64_t v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    _value.SetStr( buf );
}

void XMLAttribute::SetAttribute( float32_t v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    _value.SetStr( buf );
}


// --------- XMLElement ----------
XMLElement::XMLElement( XMLDocument* doc ) : XMLNode( doc ),
    _closingType( OPEN ),
    _rootAttribute(nullptr)
{
}

void XMLElement::DeleteAttribute( XMLAttribute* attribute )
{
    if ( attribute != nullptr )
    {
        MemPool* pool = attribute->_memPool;
        attribute->~XMLAttribute();
        pool->Free( attribute );
    }
}

XMLElement::~XMLElement()
{
    while( _rootAttribute != nullptr )
    {
        XMLAttribute* next = _rootAttribute->_next;
        DeleteAttribute( _rootAttribute );
        _rootAttribute = next;
    }
}

const XMLAttribute* XMLElement::FindAttribute( const char_t* name ) const
{
    XMLAttribute *p = nullptr;
    for( XMLAttribute* a = _rootAttribute; a != nullptr; a = a->_next )
    {
        const bool b = XMLUtil::StringEqual( a->Name(), name );
        if ( b )
        {
            p = a;
        }
    }

    return p;
}

const char_t* XMLElement::Attribute( const char_t* name, const char_t* value ) const
{
    const XMLAttribute* a = FindAttribute( name );
    if ( a == nullptr )
    {
        return nullptr;
    }

    if ( (value == nullptr) || XMLUtil::StringEqual( a->Value(), value ))
    {
        return a->Value();
    }

    return nullptr;
}

int32_t XMLElement::IntAttribute(const char_t* name, int32_t defaultValue) const
{
    int32_t i = defaultValue;
    (void)QueryIntAttribute(name, &i);
    return i;
}

uint32_t XMLElement::UnsignedAttribute(const char_t* name, uint32_t defaultValue) const
{
    uint32_t i = defaultValue;
    (void)QueryUnsignedAttribute(name, &i);
    return i;
}

int64_t XMLElement::Int64Attribute(const char_t* name, int64_t defaultValue) const
{
    int64_t i = defaultValue;
    (void)QueryInt64Attribute(name, &i);
    return i;
}

uint64_t XMLElement::Unsigned64Attribute(const char_t* name, uint64_t defaultValue) const
{
    uint64_t i = defaultValue;
    (void)QueryUnsigned64Attribute(name, &i);
    return i;
}

bool XMLElement::BoolAttribute(const char_t* name, bool defaultValue) const
{
    bool b = defaultValue;
    (void)QueryBoolAttribute(name, &b);
    return b;
}

float64_t XMLElement::DoubleAttribute(const char_t* name, float64_t defaultValue) const
{
    float64_t d = defaultValue;
    (void)QueryDoubleAttribute(name, &d);
    return d;
}

float32_t XMLElement::FloatAttribute(const char_t* name, float32_t defaultValue) const
{
    float32_t f = defaultValue;
    (void)QueryFloatAttribute(name, &f);
    return f;
}

const char_t* XMLElement::GetText() const
{
    const char_t *pCh = ( (FirstChild() != nullptr) && (FirstChild()->ToText() != nullptr) ) ? FirstChild()->Value() : nullptr;
    return pCh;
}


void XMLElement::SetText( const char_t* inText )
{
    if ( (FirstChild() != nullptr) && (FirstChild()->ToText() != nullptr) )
    {
        FirstChild()->SetValue( inText );
    }
    else
    {
        XMLText* theText = GetDocument()->NewText( inText );
        (void)(InsertFirstChild)( theText );
    }
}

void XMLElement::SetText( int32_t v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    SetText( buf );
}

void XMLElement::SetText( uint32_t v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    SetText( buf );
}

void XMLElement::SetText(int64_t v)
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr(v, &buf[0], BUF_SIZE);
    SetText(buf);
}

void XMLElement::SetText(uint64_t v)
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr(v, &buf[0], BUF_SIZE);
    SetText(buf);
}

void XMLElement::SetText( bool v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    SetText( buf );
}

void XMLElement::SetText( float32_t v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    SetText( buf );
}

void XMLElement::SetText( float64_t v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    SetText( buf );
}

XMLError XMLElement::QueryIntText( int32_t* ival ) const
{
    if ( (FirstChild() != nullptr) && (FirstChild()->ToText() != nullptr) )
    {
        const char_t* t = FirstChild()->Value();
        if ( XMLUtil::ToInt( t, ival ) )
        {
            return XML_SUCCESS;
        }

        return XML_CAN_NOT_CONVERT_TEXT;
    }

    return XML_NO_TEXT_NODE;
}

XMLError XMLElement::QueryUnsignedText( uint32_t* uval ) const
{
    if ( (FirstChild() != nullptr) && (FirstChild()->ToText() != nullptr) )
    {
        const char_t* t = FirstChild()->Value();
        if ( XMLUtil::ToUnsigned( t, uval ) )
        {
            return XML_SUCCESS;
        }

        return XML_CAN_NOT_CONVERT_TEXT;
    }

    return XML_NO_TEXT_NODE;
}

XMLError XMLElement::QueryInt64Text(int64_t* ival) const
{
    if ( (FirstChild() != nullptr) && (FirstChild()->ToText() != nullptr) )
    {
        const char_t* t = FirstChild()->Value();
        if (XMLUtil::ToInt64(t, ival))
        {
            return XML_SUCCESS;
        }

        return XML_CAN_NOT_CONVERT_TEXT;
    }

    return XML_NO_TEXT_NODE;
}

XMLError XMLElement::QueryUnsigned64Text(uint64_t* ival) const
{
    if( (FirstChild() != nullptr) && (FirstChild()->ToText() != nullptr) )
    {
        const char_t* t = FirstChild()->Value();
        if(XMLUtil::ToUnsigned64(t, ival))
        {
            return XML_SUCCESS;
        }

        return XML_CAN_NOT_CONVERT_TEXT;
    }

    return XML_NO_TEXT_NODE;
}

XMLError XMLElement::QueryBoolText( bool* bval ) const
{
    if ( (FirstChild() != nullptr) && (FirstChild()->ToText() != nullptr) )
    {
        const char_t* t = FirstChild()->Value();
        if ( XMLUtil::ToBool( t, bval ) )
        {
            return XML_SUCCESS;
        }

        return XML_CAN_NOT_CONVERT_TEXT;
    }

    return XML_NO_TEXT_NODE;
}

XMLError XMLElement::QueryDoubleText( float64_t* dval ) const
{
    if ( (FirstChild() != nullptr) && (FirstChild()->ToText() != nullptr) )
    {
        const char_t* t = FirstChild()->Value();
        if ( XMLUtil::ToDouble( t, dval ) )
        {
            return XML_SUCCESS;
        }

        return XML_CAN_NOT_CONVERT_TEXT;
    }

    return XML_NO_TEXT_NODE;
}

XMLError XMLElement::QueryFloatText( float32_t* fval ) const
{
    if ( (FirstChild() != nullptr) && (FirstChild()->ToText() != nullptr) )
    {
        const char_t* t = FirstChild()->Value();
        if ( XMLUtil::ToFloat( t, fval ) )
        {
            return XML_SUCCESS;
        }

        return XML_CAN_NOT_CONVERT_TEXT;
    }

    return XML_NO_TEXT_NODE;
}

int32_t XMLElement::IntText(int32_t defaultValue) const
{
    int32_t i = defaultValue;
    (void)QueryIntText(&i);
    return i;
}

uint32_t XMLElement::UnsignedText(uint32_t defaultValue) const
{
    uint32_t i = defaultValue;
    (void)QueryUnsignedText(&i);
    return i;
}

int64_t XMLElement::Int64Text(int64_t defaultValue) const
{
    int64_t i = defaultValue;
    (void)QueryInt64Text(&i);
    return i;
}

uint64_t XMLElement::Unsigned64Text(uint64_t defaultValue) const
{
    uint64_t i = defaultValue;
    (void)QueryUnsigned64Text(&i);
    return i;
}

bool XMLElement::BoolText(bool defaultValue) const
{
    bool b = defaultValue;
    (void)QueryBoolText(&b);
    return b;
}

float64_t XMLElement::DoubleText(float64_t defaultValue) const
{
    float64_t d = defaultValue;
    (void)QueryDoubleText(&d);
    return d;
}

float32_t XMLElement::FloatText(float32_t defaultValue) const
{
    float32_t f = defaultValue;
    (void)QueryFloatText(&f);
    return f;
}

XMLAttribute* XMLElement::CreateAttribute()
{
    TIXMLASSERT( sizeof( XMLAttribute ) == _document->_attributePool.ItemSize() );
    auto* attrib = new (_document->_attributePool.Alloc() ) XMLAttribute();
    if(attrib == nullptr)
    {
        return attrib;
    }

    TIXMLASSERT( attrib );
    attrib->_memPool = &_document->_attributePool;
    attrib->_memPool->SetTracked();
    return attrib;
}

XMLAttribute* XMLElement::FindOrCreateAttribute( const char_t* name )
{
    XMLAttribute* last = nullptr;
    XMLAttribute* attrib = nullptr;
    for( attrib = _rootAttribute;
         attrib != nullptr;
         attrib = attrib->_next )
    {
        const bool b = XMLUtil::StringEqual( attrib->Name(), name );
        if ( b )
        {
            break;
        }

        last = attrib;
    }

    if ( attrib == nullptr )
    {
        attrib = CreateAttribute();
        TIXMLASSERT( attrib );
        if ( last != nullptr )
        {
            TIXMLASSERT( last->_next == nullptr);
            last->_next = attrib;
        }
        else
        {
            TIXMLASSERT( _rootAttribute == nullptr);
            _rootAttribute = attrib;
        }

        attrib->SetName( name );
    }

    return attrib;
}

void XMLElement::DeleteAttribute( const char_t* name )
{
    XMLAttribute* prev = nullptr;
    for( XMLAttribute* a=_rootAttribute; a != nullptr; a=a->_next )
    {
        if ( XMLUtil::StringEqual( name, a->Name() ) )
        {
            if ( prev != nullptr )
            {
                prev->_next = a->_next;
            }
            else
            {
                _rootAttribute = a->_next;
            }

            DeleteAttribute( a );
            break;
        }

        prev = a;
    }
}

char_t* XMLElement::ParseAttributes( char_t* p, int32_t* curLineNumPtr )
{
    XMLAttribute* prevAttribute = nullptr;

    // Read the attributes.
    while( p != nullptr )
    {
        p = XMLUtil::SkipWhiteSpace( p, curLineNumPtr );
        if ( (*p) == '\0' )
        {
            _document->SetError( XML_ERROR_PARSING_ELEMENT, _parseLineNum, "XMLElement name=%s", Name() );
            return nullptr;
        }

        // attribute.
        const bool b = XMLUtil::IsNameStartChar( *p );
        if ( b )
        {
            XMLAttribute* attrib = CreateAttribute();
            TIXMLASSERT( attrib );
            attrib->_parseLineNum = _document->_parseCurLineNum;

            const int32_t attrLineNum = attrib->_parseLineNum;

            p = attrib->ParseDeep( p, _document->ProcessEntities(), curLineNumPtr );
            if ( (p == nullptr) || (Attribute( attrib->Name() ) != nullptr) )
            {
                DeleteAttribute( attrib );
                _document->SetError( XML_ERROR_PARSING_ATTRIBUTE, attrLineNum, "XMLElement name=%s", Name() );
                return nullptr;
            }
            // There is a minor bug here: if the attribute in the source xml
            // document is duplicated, it will not be detected and the
            // attribute will be doubly added. However, tracking the 'prevAttribute'
            // avoids re-scanning the attribute list. Preferring performance for
            // now, may reconsider in the future.
            if ( prevAttribute != nullptr )
            {
                TIXMLASSERT( prevAttribute->_next == nullptr);
                prevAttribute->_next = attrib;
            }
            else
            {
                TIXMLASSERT( _rootAttribute == nullptr);
                _rootAttribute = attrib;
            }

            prevAttribute = attrib;
        }
        // end of the tag
        else if ( (*p) == '>' )
        {
            p = &p[1];
            break;
        }
        // end of the tag
        else if ( (p[0] == '/') && (p[1] == '>') )
        {
            _closingType = CLOSED;
            return &p[2]; // done; sealed element.
        }
        else
        {
            _document->SetError( XML_ERROR_PARSING_ELEMENT, _parseLineNum, nullptr);
            return nullptr;
        }
    }

    return p;
}

XMLElement* XMLElement::InsertNewChildElement( const char_t* const name)
{
    XMLElement* node = _document->NewElement(name);
    return (InsertEndChild(node) != nullptr) ? node : nullptr;
}

XMLComment* XMLElement::InsertNewComment(const char_t* comment)
{
    XMLComment* node = _document->NewComment(comment);
    return (InsertEndChild(node) != nullptr) ? node : nullptr;
}

XMLText* XMLElement::InsertNewText(const char_t* text)
{
    XMLText* node = _document->NewText(text);
    return (InsertEndChild(node) != nullptr) ? node : nullptr;
}

XMLDeclaration* XMLElement::InsertNewDeclaration(const char_t* text)
{
    XMLDeclaration* node = _document->NewDeclaration(text);
    return (InsertEndChild(node) != nullptr) ? node : nullptr;
}

XMLUnknown* XMLElement::InsertNewUnknown(const char_t* text)
{
    XMLUnknown* node = _document->NewUnknown(text);
    return (InsertEndChild(node) != nullptr) ? node : nullptr;
}

//
//	<ele></ele>
//	<ele>foo<b>bar</b></ele>
//
char_t* XMLElement::ParseDeep( char_t* p, StrPair* parentEndTag, int32_t* curLineNumPtr )
{
    // Read the element name.
    p = XMLUtil::SkipWhiteSpace( p, curLineNumPtr );

    // The closing element is the </element> form. It is
    // parsed just like a regular element then deleted from
    // the DOM.
    if ( (*p) == '/' )
    {
        _closingType = CLOSING;
        p = &p[1];
    }

    p = _value.ParseName( p );
    if ( _value.Empty() )
    {
        return nullptr;
    }

    p = ParseAttributes( p, curLineNumPtr );
    if ( (p == nullptr) || (*p == '\0') || (_closingType != OPEN) )
    {
        return p;
    }

    p = XMLNode::ParseDeep( p, parentEndTag, curLineNumPtr );
    return p;
}

XMLNode* XMLElement::ShallowClone( XMLDocument* doc ) const
{
    if ( doc == nullptr )
    {
        doc = _document;
    }

    XMLElement* element = doc->NewElement( Value() );					// fixme: this will always allocate memory. Intern?
    for( const XMLAttribute* a=FirstAttribute(); a != nullptr; a=a->Next() )
    {
        element->SetAttribute( a->Name(), a->Value() );					// fixme: this will always allocate memory. Intern?
    }

    return element;
}

bool XMLElement::ShallowEqual( const XMLNode* compare ) const
{
    TIXMLASSERT( compare );
    const XMLElement* other = compare->ToElement();
    if ( (other != nullptr) && XMLUtil::StringEqual( other->Name(), Name() ))
    {
        const XMLAttribute* a=FirstAttribute();
        const XMLAttribute* b=other->FirstAttribute();

        while ( (a != nullptr) && (b != nullptr) )
        {
            if ( !XMLUtil::StringEqual( a->Value(), b->Value() ) )
            {
                return false;
            }

            a = a->Next();
            b = b->Next();
        }

        if ( (a != nullptr) || (b != nullptr) )
        {
            // different count
            return false;
        }

        return true;
    }

    return false;
}

bool XMLElement::Accept( XMLVisitor* visitor ) const
{
    TIXMLASSERT( visitor );
    const bool b = visitor->VisitEnter( *this, _rootAttribute );
    if ( b )
    {
        for ( const XMLNode* node=FirstChild(); node != nullptr; node=node->NextSibling() )
        {
            if ( !node->Accept( visitor ) )
            {
                break;
            }
        }
    }

    return visitor->VisitExit( *this );
}


// --------- XMLDocument -----------

// Warning: List must match 'enum XMLError'
const char_t* XMLDocument::_errorNames[XML_ERROR_COUNT] = {
    "XML_SUCCESS",
    "XML_NO_ATTRIBUTE",
    "XML_WRONG_ATTRIBUTE_TYPE",
    "XML_ERROR_FILE_NOT_FOUND",
    "XML_ERROR_FILE_COULD_NOT_BE_OPENED",
    "XML_ERROR_FILE_READ_ERROR",
    "XML_ERROR_PARSING_ELEMENT",
    "XML_ERROR_PARSING_ATTRIBUTE",
    "XML_ERROR_PARSING_TEXT",
    "XML_ERROR_PARSING_CDATA",
    "XML_ERROR_PARSING_COMMENT",
    "XML_ERROR_PARSING_DECLARATION",
    "XML_ERROR_PARSING_UNKNOWN",
    "XML_ERROR_EMPTY_DOCUMENT",
    "XML_ERROR_MISMATCHED_ELEMENT",
    "XML_ERROR_PARSING",
    "XML_CAN_NOT_CONVERT_TEXT",
    "XML_NO_TEXT_NODE",
    "XML_ELEMENT_DEPTH_EXCEEDED"
};

XMLDocument::XMLDocument( bool processEntities, Whitespace whitespaceMode ) :
    XMLNode( nullptr ),
    _writeBOM( false ),
    _processEntities( processEntities ),
    _errorID(XML_SUCCESS),
    _whitespaceMode( whitespaceMode ),
    _errorStr(),
    _errorLineNum( 0 ),
    _charBuffer(nullptr),
    _parseCurLineNum( 0 ),
    _parsingDepth(0),
    _unlinked(),
    _elementPool(),
    _attributePool(),
    _textPool(),
    _commentPool()
{
    // avoid VC++ C4355 warning about 'this' in initializer list (C4355 is off by default in VS2012+)
    _document = this;
}

void XMLDocument::DeleteNode( XMLNode* node )
{
    TIXMLASSERT( node );
    TIXMLASSERT(node->_document == this );

    if (node->_parent != nullptr)
    {
        node->_parent->DeleteChild( node );
    }
    else
    {
        // Isn't in the tree.
        // Use the parent delete.
        // Also, we need to mark it tracked: we 'know'
        // it was never used.
        node->_memPool->SetTracked();
        // Call the static XMLNode version:
        XMLNode::DeleteNode(node);
    }
}

void XMLDocument::Clear()
{
    DeleteChildren();
    while( _unlinked.Size() != 0 )
    {
        DeleteNode(_unlinked[0]);	// Will remove from _unlinked as part of delete.
    }

#ifdef TINYXML2_DEBUG
    const bool hadError = Error();
#endif
    ClearError();

    delete [] _charBuffer;
    _charBuffer = nullptr;
    _parsingDepth = 0;

#if 0
    _textPool.Trace( "text" );
    _elementPool.Trace( "element" );
    _commentPool.Trace( "comment" );
    _attributePool.Trace( "attribute" );
#endif

#ifdef TINYXML2_DEBUG
    if ( !hadError ) {
        TIXMLASSERT( _elementPool.CurrentAllocs()   == _elementPool.Untracked() );
        TIXMLASSERT( _attributePool.CurrentAllocs() == _attributePool.Untracked() );
        TIXMLASSERT( _textPool.CurrentAllocs()      == _textPool.Untracked() );
        TIXMLASSERT( _commentPool.CurrentAllocs()   == _commentPool.Untracked() );
    }
#endif
}

XMLDocument::~XMLDocument()
{
    Clear();
}

const char_t* XMLDocument::ErrorIDToName(XMLError errorID)
{
    TIXMLASSERT( errorID >= 0 && errorID < XML_ERROR_COUNT );
    const char_t* errorName = _errorNames[errorID];

    TIXMLASSERT( errorName && errorName[0] );
    return errorName;
}

void XMLDocument::SetError( XMLError error, int32_t lineNum, const char_t* format, ... )
{
    TIXMLASSERT( error >= 0 && error < XML_ERROR_COUNT );
    _errorID = error;
    _errorLineNum = lineNum;
    _errorStr.Reset();

    const size_t BUFFER_SIZE = 1000;
    char_t* buffer = new char_t[BUFFER_SIZE];
    if ( buffer == nullptr )
    {
        TIXMLASSERT(buffer == nullptr);
    }

    TIXMLASSERT(sizeof(error) <= sizeof(int32_t));
    int32_t i = TIXML_SNPRINTF(buffer, BUFFER_SIZE, "Error=%s ErrorID=%d (0x%x) Line number=%d", ErrorIDToName(error), static_cast<int32_t>(error), static_cast<int32_t>(error), lineNum);
    if ( i < 0 )
    {}

    if (format != nullptr)
    {
        size_t len = std::string(buffer).length();
        i = TIXML_SNPRINTF(&buffer[static_cast<int32_t>(len)], BUFFER_SIZE - len, ": ");
        if ( i < 0 )
        {}
        len = std::string(buffer).length();

        va_list va;
        va_start(va, format);
        i = TIXML_VSNPRINTF(&buffer[static_cast<int32_t>(len)], BUFFER_SIZE - len, format, va);
        if ( i < 0 )
        {}
        va_end(va);
    }

    _errorStr.SetStr(buffer);
    delete[] buffer;
}

void XMLDocument::MarkInUse( const XMLNode* const node)
{
    TIXMLASSERT(node);
    TIXMLASSERT(node->_parent == nullptr);

    for (int32_t i = 0; i < _unlinked.Size(); ++i)
    {
        if (node == _unlinked[i])
        {
            _unlinked.SwapRemove(i);
            break;
        }
    }
}

void XMLDocument::DeepCopy(XMLDocument* target) const
{
    TIXMLASSERT(target);
    if (target != this)
    {
        target->Clear();
        for (const XMLNode* node = this->FirstChild(); node != nullptr; node = node->NextSibling())
        {
            (void)target->InsertEndChild(node->DeepClone(target));
        }
    }
}

XMLElement* XMLDocument::NewElement( const char_t* name )
{
    auto* ele = CreateUnlinkedNode<XMLElement>( _elementPool );
    ele->SetName( name );
    return ele;
}

XMLComment* XMLDocument::NewComment( const char_t* str )
{
    auto* comment = CreateUnlinkedNode<XMLComment>( _commentPool );
    comment->SetValue( str );
    return comment;
}

XMLText* XMLDocument::NewText( const char_t* str )
{
    auto* text = CreateUnlinkedNode<XMLText>( _textPool );
    text->SetValue( str );
    return text;
}

XMLDeclaration* XMLDocument::NewDeclaration( const char_t* str )
{
    auto* dec = CreateUnlinkedNode<XMLDeclaration>( _commentPool );
    const char_t strDec[50] = {"xml version=\"1.0\" encoding=\"UTF-8\""};
    dec->SetValue( (str != nullptr) ? str : strDec);
    return dec;
}

XMLUnknown* XMLDocument::NewUnknown( const char_t* str )
{
    auto* unk = CreateUnlinkedNode<XMLUnknown>( _commentPool );
    unk->SetValue( str );
    return unk;
}

static FILE* callfopen( const char_t* filepath, const char_t* mode )
{
    TIXMLASSERT( filepath );
    TIXMLASSERT( mode );

#if defined(_MSC_VER) && (_MSC_VER >= 1400 ) && (!defined WINCE)
    FILE* fp = 0;
    const errno_t err = fopen_s( &fp, filepath, mode );
    if ( err ) {
        return 0;
    }
#else
    FILE* const fp = fopen( filepath, mode );
    if ( fp == nullptr )
    {}
#endif

    return fp;
}

void XMLDocument::Parse()
{
    TIXMLASSERT( NoChildren() ); // Clear() must have been called previously
    TIXMLASSERT( _charBuffer );

    _parseCurLineNum = 1;
    _parseLineNum = 1;

    char_t* p = _charBuffer;
    p = XMLUtil::SkipWhiteSpace( p, &_parseCurLineNum );
    p = const_cast<char_t*>( XMLUtil::ReadBOM( p, &_writeBOM ) );
    if ( (*p) == 0 )
    {
        SetError( XML_ERROR_EMPTY_DOCUMENT, 0, nullptr);
        return;
    }

    (void)ParseDeep(p, nullptr, &_parseCurLineNum );
}

XMLError XMLDocument::LoadFile( FILE* fp )
{
    Clear();

    const int32_t seekset1 = fseek( fp, 0, SEEK_SET );
    if(seekset1 != 0)
    {}

    if ( (fgetc( fp ) == EOF) && (ferror( fp ) != 0) )
    {
        SetError( XML_ERROR_FILE_READ_ERROR, 0, nullptr);
        return _errorID;
    }

    const int32_t seekend = fseek( fp, 0, SEEK_END );
    if(seekend != 0)
    {}

    const long filelength = ftell( fp );

    const int32_t seekset2 = fseek( fp, 0, SEEK_SET );
    if(seekset2 != 0)
    {}

    if ( filelength == -1L )
    {
        SetError( XML_ERROR_FILE_READ_ERROR, 0, nullptr );
        return _errorID;
    }
    TIXMLASSERT( filelength >= 0 );

    if ( !LongFitsIntoSizeTMinusOne<>::Fits( filelength ) )
    {
        // Cannot handle files which won't fit in buffer together with null terminator
        SetError( XML_ERROR_FILE_READ_ERROR, 0, nullptr);
        return _errorID;
    }

    if ( filelength == 0 )
    {
        SetError( XML_ERROR_EMPTY_DOCUMENT, 0, nullptr);
        return _errorID;
    }

    const size_t size = filelength;
    TIXMLASSERT( _charBuffer == nullptr );
    _charBuffer = new char_t[size+1];

    if (_charBuffer == nullptr)
    {
        SetError( XML_ERROR_FILE_READ_ERROR, 0, nullptr );
        return _errorID;
    }

    const size_t read = fread( _charBuffer, 1, size, fp );
    if ( read != size )
    {
        SetError( XML_ERROR_FILE_READ_ERROR, 0, nullptr );
        return _errorID;
    }

    _charBuffer[size] = '\0';

    Parse();

    return _errorID;
}

XMLError XMLDocument::LoadFile( const char_t* filename )
{
    if ( filename == nullptr )
    {
        TIXMLASSERT( false );
        SetError( XML_ERROR_FILE_COULD_NOT_BE_OPENED, 0, "filename=<nullptr>" );
        return _errorID;
    }

    Clear();

    FILE* fp = callfopen( filename, "rb" );
    if ( fp == nullptr )
    {
        SetError( XML_ERROR_FILE_NOT_FOUND, 0, "filename=%s", filename );
        return _errorID;
    }

    const XMLError error = LoadFile( fp );
    if(error == _errorID)
    {}

    const int32_t i = fclose( fp );
    if(i == -1)
    {}

    return _errorID;
}

XMLError XMLDocument::SaveFile( const FILE* const fp, bool compact )
{
    // Clear any error from the last save, otherwise it will get reported
    // for *this* call.
    (ClearError)();

    XMLPrinter stream( fp, compact );
    Print( &stream );

    return _errorID;
}

XMLError XMLDocument::SaveFile( const char_t* filename, bool compact )
{
    if ( filename == nullptr )
    {
        TIXMLASSERT( false );
        SetError( XML_ERROR_FILE_COULD_NOT_BE_OPENED, 0, "filename=<nullptr>" );
        return _errorID;
    }

    FILE* fp = callfopen( filename, "w" );
    if ( fp == nullptr )
    {
        SetError( XML_ERROR_FILE_COULD_NOT_BE_OPENED, 0, "filename=%s", filename );
        return _errorID;
    }

    (void)SaveFile(fp, compact);

    const int32_t close = fclose( fp );
    if(close != 0)
    {}

    return _errorID;
}

XMLError XMLDocument::Parse( const char_t* p, size_t len )
{
    Clear();

    if ( (len == 0) || (p == nullptr) || ((*p) == '\0') )
    {
        SetError( XML_ERROR_EMPTY_DOCUMENT, 0, nullptr);
        return _errorID;
    }

    if ( len == static_cast<size_t>(-1) )
    {
        len = std::string( p ).length();
    }

    TIXMLASSERT( _charBuffer == nullptr );
    _charBuffer = new char_t[ len+1 ];
    if( _charBuffer == nullptr)
    {
        SetError( XML_ERROR_EMPTY_DOCUMENT, 0, nullptr);
        return _errorID;
    }

    (void)memcpy( _charBuffer, p, len );
    _charBuffer[len] = '\0';

    Parse();

    if ( Error() )
    {
        // clean up now essentially dangling memory.
        // and the parse fail can put objects in the
        // pools that are dead and inaccessible.
        DeleteChildren();
        _elementPool.Clear();
        _attributePool.Clear();
        _textPool.Clear();
        _commentPool.Clear();
    }

    return _errorID;
}

void XMLDocument::Print( XMLPrinter* streamer ) const
{
    if ( streamer != nullptr )
    {
        (void)Accept( streamer );
    }
    else
    {
        XMLPrinter stdoutStreamer( stdout );
        (void)Accept( &stdoutStreamer );
    }
}

const char_t* XMLDocument::ErrorStr() const
{
    return _errorStr.Empty() ? "" : _errorStr.GetStr();
}

void XMLDocument::PrintError() const
{
    const int32_t i = printf("%s\n", ErrorStr());
    if ( i < 0 )
    {}
}

const char_t* XMLDocument::ErrorName() const
{
    return ErrorIDToName(_errorID);
}

void XMLDocument::PushDepth()
{
    _parsingDepth++;
    if (_parsingDepth == TINYXML2_MAX_ELEMENT_DEPTH)
    {
        SetError(XML_ELEMENT_DEPTH_EXCEEDED, _parseCurLineNum, "Element nesting is too deep." );
    }
}

void XMLDocument::PopDepth()
{
    TIXMLASSERT(_parsingDepth > 0);
    --_parsingDepth;
}


XMLPrinter::XMLPrinter( const FILE* const file, bool compact, int32_t depth ) : XMLVisitor(),
    _elementJustOpened( false ),
    _stack(),
    _firstElement( true ),
    _fp( const_cast<FILE*>(file) ),
    _depth( depth ),
    _textDepth( -1 ),
    _processEntities( true ),
    _compactMode( compact ),
    _buffer()
{
    for( int32_t i=0; i<ENTITY_RANGE; ++i )
    {
        _entityFlag[i] = false;
        _restrictedEntityFlag[i] = false;
    }

    for(const Entity& eachEntity : entities )
    {
        const char_t entityValue = eachEntity.value;
        const auto flagIndex = entityValue;
        TIXMLASSERT( flagIndex < ENTITY_RANGE );
        _entityFlag[static_cast<int32_t>(flagIndex)] = true;
    }

    _restrictedEntityFlag[static_cast<uint8_t>('&')] = true;
    _restrictedEntityFlag[static_cast<uint8_t>('<')] = true;
    _restrictedEntityFlag[static_cast<uint8_t>('>')] = true;	// not required, but consistency is nice
    _buffer.Push( 0 );
}

void XMLPrinter::Print( const char_t* format, ... )
{
    va_list     va;
    va_start( va, format );

    if ( _fp != nullptr )
    {
        const int32_t i = vfprintf( _fp, format, va );
        if ( i == -1 )
        {}
    }
    else
    {
        const int32_t len = TIXML_VSCPRINTF( format, va );

        // Close out and re-start the va-args
        va_end( va );
        TIXMLASSERT( len >= 0 );
        va_start( va, format );
        TIXMLASSERT( _buffer.Size() > 0 && _buffer[_buffer.Size() - 1] == 0 );

        char_t* p = _buffer.PushArr( len ) - 1;	// back up over the null terminator.
        const int32_t i = TIXML_VSNPRINTF( p, len+1, format, va );
        if ( i < 0 )
        {}
    }

    va_end( va );
}

void XMLPrinter::Write( const char_t* data, size_t size )
{
    if ( _fp != nullptr)
    {
        const size_t i = fwrite ( data , sizeof(char_t), size, _fp);
        if(i != size)
        {}
    }
    else
    {
        char_t* p = _buffer.PushArr( static_cast<int32_t>(size) ) - 1;   // back up over the null terminator.
        (void)memcpy( p, data, size );

        p[static_cast<int32_t>(size)] = '\0';
        static_cast<void>(p);
    }
}

void XMLPrinter::Putc( char_t ch )
{
    if ( _fp != nullptr )
    {
        const int32_t i = fputc ( ch, _fp);
        if ( i == -1 )
        {}
    }
    else
    {
        char_t* p = _buffer.PushArr( sizeof(char_t) ) - 1;   // back up over the null terminator.
        p[0] = ch;
        p[1] = '\0';
    }
}

void XMLPrinter::PrintSpace( int32_t depth )
{
    for( int32_t i=0; i<depth; ++i )
    {
        Write( "    " );
    }
}

void XMLPrinter::PrintString( const char_t* p, bool restricted )
{
    // Look for runs of bytes between entities to print.
    const char_t* q = p;

    if ( _processEntities )
    {
        const bool* flag = (restricted ? _restrictedEntityFlag : _entityFlag);
        while ( (*q) != 0 )
        {
            TIXMLASSERT( p <= q );
            // Remember, char is sometimes signed. (How many times has that bitten me?)
            if ( (*q > 0) && (*q < ENTITY_RANGE) )
            {
                // Check for entities. If one is found, flush
                // the stream up until the entity, write the
                // entity, and keep looking.
                if ( flag[static_cast<int32_t>(*q)] )
                {
                    while ( p < q )
                    {
                        const size_t delta = q - p;
                        const size_t toPrint = ( INT_MAX < delta ) ? INT_MAX : delta;
                        Write( p, toPrint );
                        p = &p[toPrint];
                    }

                    bool entityPatternPrinted = false;
                    for(const Entity& eachEntity : entities)
                    {
                        if ( eachEntity.value == *q )
                        {
                            Putc( '&' );
                            Write( eachEntity.pattern, eachEntity.length );
                            Putc( ';' );
                            entityPatternPrinted = true;
                            break;
                        }
                    }

                    if ( !entityPatternPrinted )
                    {
                        // TIXMLASSERT( entityPatternPrinted ) causes gcc -Wunused-but-set-variable in release
                        TIXMLASSERT( false );
                    }

                    p = &p[1];
                }
            }

            q = &q[1];
            TIXMLASSERT( p <= q );
        }

        // Flush the remaining string. This will be the entire
        // string if an entity wasn't found.
        if ( p < q )
        {
            const size_t delta = q - p;
            const size_t toPrint = ( INT_MAX < delta ) ? INT_MAX : delta;
            Write( p, toPrint );
        }
    }
    else
    {
        Write( p );
    }
}

void XMLPrinter::SealElementIfJustOpened()
{
    if ( _elementJustOpened )
    {
        _elementJustOpened = false;
        Putc( '>' );
    }
}

void XMLPrinter::PushDeclaration( const char_t* value )
{
    SealElementIfJustOpened();

    if ( (_textDepth < 0) && (!_firstElement) && (!_compactMode))
    {
        Putc( '\n' );
        PrintSpace( _depth );
    }

    _firstElement = false;

    Write( "<?" );
    Write( value );
    Write( "?>" );
}

void XMLPrinter::PushHeader( bool writeBOM, bool writeDec )
{
    if ( writeBOM )
    {
        static const uint8_t staticBom[] = { TIXML_UTF_LEAD_0, TIXML_UTF_LEAD_1, TIXML_UTF_LEAD_2, 0 };
        Write( reinterpret_cast< const char_t* >( staticBom ) );
    }

    if ( writeDec )
    {
        PushDeclaration( "xml version=\"1.0\"" );
    }
}

void XMLPrinter::OpenElement( const char_t* name, bool compactMode )
{
    (SealElementIfJustOpened)();
    _stack.Push( name );

    if ( (_textDepth < 0) && (!_firstElement) && (!compactMode) )
    {
        Putc( '\n' );
        PrintSpace( _depth );
    }

    Write ( "<" );
    Write ( name );

    _elementJustOpened = true;
    _firstElement = false;
    ++_depth;
}

void XMLPrinter::PushAttribute( const char_t* name, const char_t* value )
{
    TIXMLASSERT( _elementJustOpened );
    Putc ( ' ' );
    Write( name );
    Write( "=\"" );
    PrintString( value, false );
    Putc ( '\"' );
}

void XMLPrinter::PushAttribute( const char_t* name, int32_t v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    PushAttribute( name, buf );
}

void XMLPrinter::PushAttribute( const char_t* name, uint32_t v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    PushAttribute( name, buf );
}

void XMLPrinter::PushAttribute(const char_t* name, int64_t v)
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr(v, &buf[0], BUF_SIZE);
    PushAttribute(name, buf);
}

void XMLPrinter::PushAttribute(const char_t* name, uint64_t v)
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr(v, &buf[0], BUF_SIZE);
    PushAttribute(name, buf);
}

void XMLPrinter::PushAttribute( const char_t* name, bool v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    PushAttribute( name, buf );
}

void XMLPrinter::PushAttribute( const char_t* name, float64_t v )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( v, &buf[0], BUF_SIZE );
    PushAttribute( name, buf );
}

void XMLPrinter::CloseElement( bool compactMode )
{
    --_depth;
    const char_t* name = _stack.Pop();

    if ( _elementJustOpened )
    {
        Write( "/>" );
    }
    else
    {
        if ( (_textDepth < 0) && (!compactMode))
        {
            Putc( '\n' );
            PrintSpace( _depth );
        }

        Write ( "</" );
        Write ( name );
        Write ( ">" );
    }

    if ( _textDepth == _depth )
    {
        _textDepth = -1;
    }

    if ( (_depth == 0) && (!compactMode))
    {
        Putc( '\n' );
    }

    _elementJustOpened = false;
}

void XMLPrinter::PushText( const char_t* text, bool cdata )
{
    _textDepth = _depth-1;

    SealElementIfJustOpened();
    if ( cdata )
    {
        Write( "<![CDATA[" );
        Write( text );
        Write( "]]>" );
    }
    else
    {
        PrintString( text, true );
    }
}

void XMLPrinter::PushText( int64_t value )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( value, &buf[0], BUF_SIZE );
    PushText( buf, false );
}

void XMLPrinter::PushText( uint64_t value )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr(value, &buf[0], BUF_SIZE);
    PushText(buf, false);
}

void XMLPrinter::PushText( int32_t value )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( value, &buf[0], BUF_SIZE );
    PushText( buf, false );
}

void XMLPrinter::PushText( uint32_t value )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( value, &buf[0], BUF_SIZE );
    PushText( buf, false );
}

void XMLPrinter::PushText( bool value )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( value, &buf[0], BUF_SIZE );
    PushText( buf, false );
}

void XMLPrinter::PushText( float32_t value )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( value, &buf[0], BUF_SIZE );
    PushText( buf, false );
}

void XMLPrinter::PushText( float64_t value )
{
    char_t buf[BUF_SIZE];
    XMLUtil::ToStr( value, &buf[0], BUF_SIZE );
    PushText( buf, false );
}

void XMLPrinter::PushComment( const char_t* comment )
{
    SealElementIfJustOpened();
    if ( (_textDepth < 0) && (!_firstElement) && (!_compactMode))
    {
        Putc( '\n' );
        PrintSpace( _depth );
    }

    _firstElement = false;

    Write( "<!--" );
    Write( comment );
    Write( "-->" );
}

void XMLPrinter::PushUnknown( const char_t* value )
{
    SealElementIfJustOpened();
    if ( (_textDepth < 0) && (!_firstElement) && (!_compactMode))
    {
        Putc( '\n' );
        PrintSpace( _depth );
    }

    _firstElement = false;

    Write( "<!" );
    Write( value );
    Putc( '>' );
}

bool XMLPrinter::VisitEnter( const XMLDocument& doc )
{
    _processEntities = doc.ProcessEntities();
    if ( doc.HasBOM() )
    {
        PushHeader( true, false );
    }

    return true;
}

bool XMLPrinter::VisitEnter( const XMLElement& element, const XMLAttribute* attribute )
{
    const XMLElement* parentElem = nullptr;
    if ( element.Parent() != nullptr )
    {
        parentElem = element.Parent()->ToElement();
    }

    const bool compactMode = (parentElem != nullptr) ? CompactMode( *parentElem ) : _compactMode;

    OpenElement( element.Name(), compactMode );

    while ( attribute != nullptr )
    {
        PushAttribute( attribute->Name(), attribute->Value() );
        attribute = attribute->Next();
    }

    return true;
}

bool XMLPrinter::VisitExit( const XMLElement& element )
{
    CloseElement( CompactMode(element) );
    return true;
}

bool XMLPrinter::Visit( const XMLText& text )
{
    PushText( text.Value(), text.CData() );
    return true;
}

bool XMLPrinter::Visit( const XMLComment& comment )
{
    PushComment( comment.Value() );
    return true;
}

bool XMLPrinter::Visit( const XMLDeclaration& declaration )
{
    PushDeclaration( declaration.Value() );
    return true;
}

bool XMLPrinter::Visit( const XMLUnknown& unknown )
{
    PushUnknown( unknown.Value() );
    return true;
}

void XMLPrinter::Write(const char_t *data)
{
    (Write)(data, std::string(data).length());
}

bool XMLVisitor::VisitExit(const XMLDocument &)
{
    return true;
}

bool XMLVisitor::VisitExit(const XMLElement &)
{
    return true;
}

bool XMLVisitor::Visit(const XMLDeclaration &)
{
    return true;
}

bool XMLVisitor::Visit(const XMLText &)
{
    return true;
}

bool XMLVisitor::Visit(const XMLComment &)
{
    return true;
}

bool XMLVisitor::Visit(const XMLUnknown &)
{
    return true;
}

bool XMLVisitor::VisitEnter(const XMLDocument &)
{
    return true;
}

bool XMLVisitor::VisitEnter(const XMLElement &, const XMLAttribute *)
{
    return true;
}

XMLComment *XMLNode::ToComment()
{
    return nullptr;
}

const XMLComment *XMLNode::ToComment() const
{
    return nullptr;
}

XMLDeclaration *XMLNode::ToDeclaration()
{
    return nullptr;
}

const XMLDeclaration *XMLNode::ToDeclaration() const
{
    return nullptr;
}

XMLUnknown *XMLNode::ToUnknown()
{
    return nullptr;
}

const XMLUnknown *XMLNode::ToUnknown() const
{
    return nullptr;
}

XMLText *XMLNode::ToText()
{
    return nullptr;
}

const XMLText *XMLNode::ToText() const
{
    return nullptr;
}

XMLDocument *XMLNode::ToDocument()
{
    return nullptr;
}

const XMLDocument *XMLNode::ToDocument() const
{
    return nullptr;
}

XMLElement *XMLNode::ToElement()
{
    return nullptr;
}

}   // namespace simproxml
