﻿/*
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

#ifndef SIMPROXML_INCLUDED
#define SIMPROXML_INCLUDED

#if defined(ANDROID_NDK) || defined(__BORLANDC__) || defined(__QNXNTO__)
#   include <ctype.h>
#   include <limits.h>
#   include <stdio.h>
#   include <stdlib.h>
#   include <string.h>
#	if defined(__PS3__)
#		include <stddef.h>
#	endif
#else
#   include <cctype>
#   include <climits>
//#   include <cstdio>
#   include <cstdlib>
#   include <cstring>
#   include <string>
#   include <algorithm>
#endif

#include "../common.h"


/*
   TODO: intern strings instead of allocation.
*/
/*
        gcc:
        g++ -Wall -DTINYXML2_DEBUG tinyxml2.cpp xmltest.cpp -o gccxmltest.exe

    Formatting, Artistic Style:
        AStyle.exe --style=1tbs --indent-switches --break-closing-brackets --indent-preprocessor tinyxml2.cpp tinyxml2.h
*/

#if defined( _DEBUG ) || defined (__DEBUG__)
#   ifndef TINYXML2_DEBUG
#       define TINYXML2_DEBUG
#   endif
#endif

#ifdef _MSC_VER
#   pragma warning(push)
#   pragma warning(disable: 4251)
#endif

#ifdef _WIN32
#   ifdef TINYXML2_EXPORT
#       define TINYXML2_LIB __declspec(dllexport)
#   elif defined(TINYXML2_IMPORT)
#       define TINYXML2_LIB __declspec(dllimport)
#   else
#       define TINYXML2_LIB
#   endif
#elif __GNUC__ >= 4
#   define TINYXML2_LIB __attribute__((visibility("default")))
#else
#   define TINYXML2_LIB
#endif


#if defined(TINYXML2_DEBUG)
#   if defined(_MSC_VER)
#       // "(void)0," is for suppressing C4127 warning in "assert(false)", "assert(true)" and the like
#       define TIXMLASSERT( x )           if ( !((void)0,(x))) { __debugbreak(); }
#   elif defined (ANDROID_NDK)
#       include <android/log.h>
#       define TIXMLASSERT( x )           if ( !(x)) { __android_log_assert( "assert", "grinliz", "ASSERT in '%s' at %d.", __FILE__, __LINE__ ); }
#   else
#       include <cassert>
#       define TIXMLASSERT                assert
#   endif
#else
#   define TIXMLASSERT( x )               {}
#endif


/* Versioning, past 1.0.14:
        http://semver.org/
*/
//static const int TIXML2_MAJOR_VERSION = 8;
//static const int TIXML2_MINOR_VERSION = 0;
//static const int TIXML2_PATCH_VERSION = 0;

//#define TINYXML2_MAJOR_VERSION 8
//#define TINYXML2_MINOR_VERSION 0
//#define TINYXML2_PATCH_VERSION 0

namespace simproxml
{
class XMLDocument;
class XMLElement;
class XMLAttribute;
class XMLComment;
class XMLText;
class XMLDeclaration;
class XMLUnknown;
class XMLPrinter;

/*
        A class that wraps strings. Normally stores the start and end
        pointers into the XML file itself, and will apply normalization
        and entity translation if actually read. Can also store (and memory
        manage) a traditional char[]

    Isn't clear why TINYXML2_LIB is needed; but seems to fix #719
*/
class TINYXML2_LIB StrPair
{
public:
    enum
    {
        NEEDS_ENTITY_PROCESSING			= 0x01,
        NEEDS_NEWLINE_NORMALIZATION		= 0x02,
        NEEDS_WHITESPACE_COLLAPSING     = 0x04,

        TEXT_ELEMENT		            = NEEDS_ENTITY_PROCESSING | NEEDS_NEWLINE_NORMALIZATION,
        TEXT_ELEMENT_LEAVE_ENTITIES		= NEEDS_NEWLINE_NORMALIZATION,
        ATTRIBUTE_NAME		            = 0,
        ATTRIBUTE_VALUE		            = NEEDS_ENTITY_PROCESSING | NEEDS_NEWLINE_NORMALIZATION,
        ATTRIBUTE_VALUE_LEAVE_ENTITIES  = NEEDS_NEWLINE_NORMALIZATION,
        COMMENT							= NEEDS_NEWLINE_NORMALIZATION
    };

    StrPair() : _flags( 0 ), _start(nullptr), _end(nullptr) {}
    ~StrPair();

    void Reset();
    void Set( char_t* start, char_t* end, int32_t flags )
    {
        TIXMLASSERT( start );
        TIXMLASSERT( end );

        Reset();
        _start  = start;
        _end    = end;
        _flags  = (static_cast<uint32_t>(flags) | static_cast<uint32_t>(NEEDS_FLUSH));
    }

    const char_t* GetStr();
    void SetStr( const char_t* str, int32_t flags = 0 );

    bool Empty() const
    {
        return _start == _end;
    }

    void SetInternedStr( const char_t* str )
    {
        Reset();
        _start = const_cast<char_t*>(str);
    }

    char_t* ParseText( char_t* p, const char_t* endTag, int32_t strFlags, int32_t* curLineNumPtr );
    char_t* ParseName( char_t* p );

    void TransferTo( StrPair* other );

private:
    void CollapseWhitespace();

    enum
    {
        NEEDS_FLUSH = 0x100U,
        NEEDS_DELETE = 0x200U
    };

    uint32_t  _flags;
    char_t*   _start;
    char_t*   _end;

    StrPair( const StrPair& other ) = delete;	// not supported
    void operator=( const StrPair& other ) = delete;	// not supported, use TransferTo()
};


/*
        A dynamic array of Plain Old Data. Doesn't support constructors, etc.
        Has a small initial memory pool, so that low or no usage will not
        cause a call to new/delete
*/
template <class T, int32_t INITIAL_SIZE>
class DynArray
{
public:
    DynArray() :
        _allocated( INITIAL_SIZE ),
        _size( 0 )
    {
        std::fill(&_pool[0], &_pool[0]+INITIAL_SIZE, T());
        _mem = _pool;
    }

    ~DynArray()
    {
        if ( _mem != _pool )
        {
            delete [] _mem;
        }
    }

    void Clear()
    {
        _size = 0;
    }

    void Push( T t )
    {
        TIXMLASSERT( _size < INT_MAX );

        EnsureCapacity( _size+1 );

        _mem[_size] = t;
        ++_size;
    }

    T* PushArr( int32_t count )
    {
        TIXMLASSERT( count >= 0 );
        TIXMLASSERT( _size <= INT_MAX - count );

        EnsureCapacity( _size+count );

        T* ret = &_mem[_size];
        _size += count;

        return ret;
    }

    T Pop()
    {
        TIXMLASSERT( _size > 0 );

        --_size;

        return _mem[_size];
    }

    void PopArr( int32_t count )
    {
        TIXMLASSERT( _size >= count );

        _size -= count;
    }

    bool Empty() const
    {
        return _size == 0;
    }

//    T& operator[](int32_t i)				{
//        TIXMLASSERT( i>= 0 && i < _size );
//        return _mem[i];
//    }

    const T& operator[](int32_t i) const
    {
        TIXMLASSERT( i>= 0 && i < _size );

        return _mem[i];
    }

    const T& PeekTop() const
    {
        TIXMLASSERT( _size > 0 );

        return _mem[_size - 1];
    }

    int32_t Size() const
    {
        TIXMLASSERT( _size >= 0 );

        return _size;
    }

    int32_t Capacity() const
    {
        TIXMLASSERT( _allocated >= INITIAL_SIZE );

        return _allocated;
    }

    void SwapRemove(int32_t i)
    {
        TIXMLASSERT(i >= 0 && i < _size);
        TIXMLASSERT(_size > 0);

        _mem[i] = _mem[_size - 1];
        --_size;
    }

    const T* Mem() const
    {
        TIXMLASSERT( _mem );

        return _mem;
    }

    T* Mem()
    {
        TIXMLASSERT( _mem );

        return _mem;
    }

private:
    DynArray( const DynArray& ) = delete; // not supported
    void operator=( const DynArray& ) = delete; // not supported

    void EnsureCapacity( int32_t cap )
    {
        TIXMLASSERT( cap > 0 );

        if ( cap > _allocated )
        {
            TIXMLASSERT( cap <= INT_MAX / 2 );

            const int32_t newAllocated = cap * 2;
            T* newMem = new T[newAllocated];

            TIXMLASSERT( newAllocated >= _size );

            (void)memcpy( newMem, _mem, sizeof(T)*_size );	// warning: not using constructors, only works for PODs
            if ( _mem != _pool )
            {
                delete [] _mem;
            }
            _mem = newMem;
            _allocated = newAllocated;
        }
    }

    T*  _mem;
    T   _pool[INITIAL_SIZE];
    int32_t _allocated;		// objects allocated
    int32_t _size;			// number objects in use
};


/*
        Parent virtual class of a pool for fast allocation
        and deallocation of objects.
*/
class MemPool
{
public:
    MemPool() = default;
    virtual ~MemPool() = default;

    virtual int32_t ItemSize() const = 0;
    virtual void* Alloc() = 0;
    virtual void Free(const void* ) = 0;
    virtual void SetTracked() = 0;

private:
    MemPool& operator=( const MemPool& ) = delete; // not supported
};


/*
        Template child class to create pools of the correct type.
*/
template< int32_t ITEM_SIZE >
class MemPoolT : public MemPool
{
public:
    MemPoolT() : MemPool(), _blockPtrs(), _root(nullptr), _currentAllocs(0), _nAllocs(0), _maxAllocs(0), _nUntracked(0)	{}
    virtual ~MemPoolT()
    {
        MemPoolT< ITEM_SIZE >::Clear();
    }

    void Clear()
    {
        // Delete the blocks.
        while( _blockPtrs.Empty() == false )
        {
            Block* lastBlock = _blockPtrs.Pop();
            if (lastBlock != nullptr)
            {
                delete lastBlock;
                lastBlock = nullptr;
            }
        }

        _root = nullptr;
        _currentAllocs = 0;
        _nAllocs = 0;
        _maxAllocs = 0;
        _nUntracked = 0;
    }

    virtual int32_t ItemSize() const
    {
        return ITEM_SIZE;
    }

    int32_t CurrentAllocs() const
    {
        return _currentAllocs;
    }

    virtual void* Alloc()
    {
        if ( _root == nullptr )
        {
            // Need a new block.
            auto* block = new Block();
            _blockPtrs.Push( block );

            Item* blockItems = block->items;

            int32_t j(ITEMS_PER_BLOCK - 1);
            for( int32_t i = 0; (i < j); ++i )
            {
                blockItems[i].next = &(blockItems[i + 1]);
            }

            blockItems[ITEMS_PER_BLOCK - 1].next = nullptr;
            _root = blockItems;
        }

        Item* const result = _root;
        TIXMLASSERT( result != nullptr);
        _root = _root->next;

        ++_currentAllocs;
        if ( _currentAllocs > _maxAllocs )
        {
            _maxAllocs = _currentAllocs;
        }
        ++_nAllocs;
        ++_nUntracked;
        return result;
    }

    virtual void Free(const void* mem )
    {
        if ( nullptr == mem )
        {
            return;
        }

        --_currentAllocs;
        Item* item = static_cast<Item*>(const_cast<void*>(mem) );
#ifdef TINYXML2_DEBUG
        memset( item, 0xfe, sizeof( *item ) );
#endif
        item->next = _root;
        _root = item;
    }

    void Trace( const char_t* name )
    {
        printf( "Mempool %s watermark=%d [%dk] current=%d size=%d nAlloc=%d blocks=%d\n",
                name, _maxAllocs, _maxAllocs * ITEM_SIZE / 1024, _currentAllocs,
                ITEM_SIZE, _nAllocs, _blockPtrs.Size()
              );
    }

    virtual void SetTracked()
    {
        --_nUntracked;
    }

    int32_t Untracked() const
    {
        return _nUntracked;
    }

    // This number is perf sensitive. 4k seems like a good tradeoff on my machine.
    // The test file is large, 170k.
    // Release:		VS2010 gcc(no opt)
    //		1k:		4000
    //		2k:		4000
    //		4k:		3900	21000
    //		16k:	5200
    //		32k:	4300
    //		64k:	4000	21000
    // Declared public because some compilers do not accept to use ITEMS_PER_BLOCK
    // in private part if ITEMS_PER_BLOCK is private
    enum { ITEMS_PER_BLOCK = (4 * 1024) / ITEM_SIZE };

private:
    MemPoolT( const MemPoolT& ) = delete; // not supported
    void operator=( const MemPoolT& ) = delete; // not supported

    struct Item
    {
        Item*   next;
        char_t    itemData[ITEM_SIZE];
    };

    struct Block
    {
        MemPoolT::Item items[ITEMS_PER_BLOCK];
    };

    DynArray< Block*, 10 > _blockPtrs;
    Item* _root;

    int32_t _currentAllocs;
    int32_t _nAllocs;
    int32_t _maxAllocs;
    int32_t _nUntracked;
};



/**
        Implements the interface to the "Visitor pattern" (see the Accept() method.)
        If you call the Accept() method, it requires being passed a XMLVisitor
        class to handle callbacks. For nodes that contain other nodes (Document, Element)
        you will get called with a VisitEnter/VisitExit pair. Nodes that are always leafs
        are simply called with Visit().

        If you return 'true' from a Visit method, recursive parsing will continue. If you return
        false, <b>no children of this node or its siblings</b> will be visited.

        All flavors of Visit methods have a default implementation that returns 'true' (continue
        visiting). You need to only override methods that are interesting to you.

        Generally Accept() is called on the XMLDocument, although all nodes support visiting.

        You should never change the document from a callback.

        @sa XMLNode::Accept()
*/
class TINYXML2_LIB XMLVisitor
{
public:
    virtual ~XMLVisitor() = default;

    /// Visit a document.
    virtual bool VisitEnter( const XMLDocument& /*doc*/ );
    /// Visit a document.
    virtual bool VisitExit( const XMLDocument& /*doc*/ );

    /// Visit an element.
    virtual bool VisitEnter( const XMLElement& /*element*/, const XMLAttribute* /*firstAttribute*/ );
    /// Visit an element.
    virtual bool VisitExit( const XMLElement& /*element*/ );

    /// Visit a declaration.
    virtual bool Visit( const XMLDeclaration& /*declaration*/ );
    /// Visit a text node.
    virtual bool Visit( const XMLText& /*text*/ );
    /// Visit a comment node.
    virtual bool Visit( const XMLComment& /*comment*/ );
    /// Visit an unknown node.
    virtual bool Visit( const XMLUnknown& /*unknown*/ );
};

// WARNING: must match XMLDocument::_errorNames[]
enum XMLError
{
    XML_SUCCESS = 0,
    XML_NO_ATTRIBUTE,
    XML_WRONG_ATTRIBUTE_TYPE,
    XML_ERROR_FILE_NOT_FOUND,
    XML_ERROR_FILE_COULD_NOT_BE_OPENED,
    XML_ERROR_FILE_READ_ERROR,
    XML_ERROR_PARSING_ELEMENT,
    XML_ERROR_PARSING_ATTRIBUTE,
    XML_ERROR_PARSING_TEXT,
    XML_ERROR_PARSING_CDATA,
    XML_ERROR_PARSING_COMMENT,
    XML_ERROR_PARSING_DECLARATION,
    XML_ERROR_PARSING_UNKNOWN,
    XML_ERROR_EMPTY_DOCUMENT,
    XML_ERROR_MISMATCHED_ELEMENT,
    XML_ERROR_PARSING,
    XML_CAN_NOT_CONVERT_TEXT,
    XML_NO_TEXT_NODE,
    XML_ELEMENT_DEPTH_EXCEEDED,

    XML_ERROR_COUNT
};


/*
        Utility functionality.
*/
class TINYXML2_LIB XMLUtil
{
public:
    static const char_t* SkipWhiteSpace( const char_t* p, int32_t* curLineNumPtr )
    {
        TIXMLASSERT( p );

        while( IsWhiteSpace(*p) )
        {
            if ((curLineNumPtr != nullptr) && ((*p) == '\n'))
            {
                ++(*curLineNumPtr);
            }

            p = &p[1];
        }

        TIXMLASSERT( p );
        return p;
    }

    static char_t* SkipWhiteSpace( char_t* const p, int* const curLineNumPtr )
    {
        return const_cast<char_t*>( SkipWhiteSpace( const_cast<const char_t*>(p), curLineNumPtr ) );
    }

    // Anything in the high order range of UTF-8 is assumed to not be whitespace. This isn't
    // correct, but simple, and usually works.
    static bool IsWhiteSpace( char_t p )
    {
        return (!IsUTF8Continuation(static_cast<int32_t>(p))) && ((bool)isspace( static_cast<uint8_t>(p) ));
    }

    inline static bool IsNameStartChar( uint8_t ch )
    {
        const bool b = ( (static_cast<uint32_t>(ch) >= 128) || (isalpha( ch ) != 0 )) ? true : ((ch == ':') || (ch == '_'));
        return b;
    }

    inline static bool IsNameChar( char_t ch )
    {
        return (
                IsNameStartChar( ch )
                || ( isdigit( ch ) != 0 )
                || ( ch == '.' )
                || ( ch == '-' )
               );
    }

    inline static bool StringEqual( const char_t* p, const char_t* q, int32_t nChar=INT_MAX )
    {
        if ( p == q )
        {
            return true;
        }

        TIXMLASSERT( p );
        TIXMLASSERT( q );
        TIXMLASSERT( nChar >= 0 );
        return strncmp( p, q, nChar ) == 0;
    }

    inline static bool IsUTF8Continuation( const int32_t p )
    {
        return ( static_cast<int32_t>(p) & 0x80 ) != 0;
    }

    static const char_t* ReadBOM( const char_t* p, bool* bom );

    // p is the starting location,
    // the UTF-8 value of the entity will be placed in value, and length filled in.
    static const char_t* GetCharacterRef( const char_t* p, char_t* value, int32_t* length );
    static void ConvertUTF32ToUTF8( uint64_t input, char_t* output, int32_t* length );

    // converts primitive types to strings
    static void ToStr( bool      v, char_t* const buffer, int32_t bufferSize );
    static void ToStr( int32_t   v, char_t* const buffer, int32_t bufferSize );
    static void ToStr( uint32_t  v, char_t* const buffer, int32_t bufferSize );
    static void ToStr( float32_t v, char_t* const buffer, int32_t bufferSize );
    static void ToStr( float64_t v, char_t* const buffer, int32_t bufferSize );
    static void ToStr( int64_t   v, char_t* const buffer, int32_t bufferSize );
    static void ToStr( uint64_t  v, char_t* const buffer, int32_t bufferSize );

    // converts strings to primitive types
    static bool	ToBool(       const char_t* const str, bool* const value );
    static bool	ToInt(        const char_t* const str, int32_t*    value );
    static bool ToUnsigned(   const char_t* const str, uint32_t*   value );
    static bool	ToFloat(      const char_t*       str, float32_t*  value );
    static bool ToDouble(     const char_t*       str, float64_t*  value );
    static bool ToInt64(      const char_t*       str, int64_t*    value );
    static bool ToUnsigned64( const char_t*       str, uint64_t*   value );
    // Changes what is serialized for a boolean value.
    // Default to "true" and "false". Shouldn't be changed
    // unless you have a special testing or compatibility need.
    // Be careful: static, global, & not thread safe.
    // Be sure to set static const memory as parameters.
    static void SetBoolSerialization(const char_t* writeTrue, const char_t* writeFalse);

private:
    static const char_t* writeBoolTrue;
    static const char_t* writeBoolFalse;
};


/** XMLNode is a base class for every object that is in the
        XML Document Object Model (DOM), except XMLAttributes.
        Nodes have siblings, a parent, and children which can
        be navigated. A node is always in a XMLDocument.
        The type of a XMLNode can be queried, and it can
        be cast to its more defined type.

        A XMLDocument allocates memory for all its Nodes.
        When the XMLDocument gets deleted, all its Nodes
        will also be deleted.

        @verbatim
        A Document can contain:	Element	(container or leaf)
                                                        Comment (leaf)
                                                        Unknown (leaf)
                                                        Declaration( leaf )

        An Element can contain:	Element (container or leaf)
                                                        Text	(leaf)
                                                        Attributes (not on tree)
                                                        Comment (leaf)
                                                        Unknown (leaf)

        @endverbatim
*/
class TINYXML2_LIB XMLNode
{
    friend class XMLDocument;
    friend class XMLElement;

public:
    /// Get the XMLDocument that owns this XMLNode.
    const XMLDocument* GetDocument() const
    {
        TIXMLASSERT( _document );
        return _document;
    }

    /// Get the XMLDocument that owns this XMLNode.
    XMLDocument* GetDocument()
    {
        TIXMLASSERT( _document );
        return _document;
    }

    /// Safely cast to Text, or null.
    virtual XMLText*		ToText();
    /// Safely cast to a Comment, or null.
    virtual XMLComment*		ToComment();
    /// Safely cast to a Document, or null.
    virtual XMLDocument*	ToDocument();
    /// Safely cast to a Declaration, or null.
    virtual XMLDeclaration*	ToDeclaration();
    /// Safely cast to an Unknown, or null.
    virtual XMLUnknown*		ToUnknown();
    /// Safely cast to an Element, or null.
    virtual XMLElement*		ToElement();

    virtual const XMLElement*		ToElement() const
    {
        return nullptr;
    }
    virtual const XMLText*			ToText() const;
    virtual const XMLComment*		ToComment() const;
    virtual const XMLDocument*		ToDocument() const;
    virtual const XMLDeclaration*	ToDeclaration() const;
    virtual const XMLUnknown*		ToUnknown() const;

    /** The meaning of 'value' changes for the specific type.
        @verbatim
        Document:	empty (NULL is returned, not an empty string)
        Element:	name of the element
        Comment:	the comment text
        Unknown:	the tag contents
        Text:		the text string
        @endverbatim
    */
    const char_t* Value() const;
    /** Set the Value of an XML node.
        @sa Value()
    */
    void SetValue( const char_t* str, bool staticMem=false );

    /// Gets the line number the node is in, if the document was parsed from a file.
    int32_t GetLineNum() const
    {
        return GetParseLineNum();
    }

    /// Get the parent of this node on the DOM.
    const XMLNode*	Parent() const
    {
        return _parent;
    }
    XMLNode* Parent()
    {
        return _parent;
    }

    /// Returns true if this node has no children.
    bool NoChildren() const
    {
        return (_firstChild == nullptr);
    }

    /// Get the first child node, or null if none exists.
    const XMLNode*  FirstChild() const
    {
        return _firstChild;
    }
    XMLNode*		FirstChild()
    {
        return _firstChild;
    }

    /** Get the first child element, or optionally the first child
        element with the specified name.
    */
    const XMLElement* FirstChildElement( const char_t* const name = nullptr) const;
    XMLElement* FirstChildElement( const char* name = nullptr )
    {
        return const_cast<XMLElement*>(const_cast<const XMLNode*>(this)->FirstChildElement( name ));
    }

    /// Get the last child node, or null if none exists.
    const XMLNode*	LastChild() const
    {
        return _lastChild;
    }
    XMLNode*		LastChild()
    {
        return _lastChild;
    }

    /** Get the last child element or optionally the last child
        element with the specified name.
    */
    const XMLElement* LastChildElement( const char_t* const name = nullptr) const;
    XMLElement* LastChildElement( const char* name = nullptr )
    {
        return const_cast<XMLElement*>(const_cast<const XMLNode*>(this)->LastChildElement(name) );
    }

    /// Get the previous (left) sibling node of this node.
    const XMLNode*	PreviousSibling() const
    {
        return _prev;
    }
    XMLNode*	PreviousSibling()
    {
        return _prev;
    }

    /// Get the previous (left) sibling element of this node, with an optionally supplied name.
    const XMLElement*	PreviousSiblingElement( const char_t* const name = nullptr) const;
    XMLElement*	PreviousSiblingElement( const char* name = nullptr )
    {
        return const_cast<XMLElement*>(const_cast<const XMLNode*>(this)->PreviousSiblingElement( name ) );
    }

    /// Get the next (right) sibling node of this node.
    const XMLNode*	NextSibling() const
    {
        return _next;
    }
    XMLNode*	NextSibling()
    {
        return _next;
    }

    /// Get the next (right) sibling element of this node, with an optionally supplied name.
    const XMLElement*	NextSiblingElement( const char_t* const name = nullptr) const;
    XMLElement*	NextSiblingElement( const char* name = nullptr)
    {
        return const_cast<XMLElement*>(const_cast<const XMLNode*>(this)->NextSiblingElement( name ) );
    }

    /**
        Add a child node as the last (right) child.
                If the child node is already part of the document,
                it is moved from its old location to the new location.
                Returns the addThis argument or 0 if the node does not
                belong to the same document.
    */
    XMLNode* InsertEndChild( XMLNode* addThis );

    XMLNode* LinkEndChild( XMLNode* addThis )
    {
        return (InsertEndChild)( addThis );
    }
    /**
        Add a child node as the first (left) child.
                If the child node is already part of the document,
                it is moved from its old location to the new location.
                Returns the addThis argument or 0 if the node does not
                belong to the same document.
    */
    XMLNode* InsertFirstChild( XMLNode* addThis );
    /**
        Add a node after the specified child node.
                If the child node is already part of the document,
                it is moved from its old location to the new location.
                Returns the addThis argument or 0 if the afterThis node
                is not a child of this node, or if the node does not
                belong to the same document.
    */
    XMLNode* InsertAfterChild( XMLNode* afterThis, XMLNode* addThis );

    /**
        Delete all the children of this node.
    */
    void DeleteChildren();

    /**
        Delete a child of this node.
    */
    void DeleteChild( XMLNode* node );

    /**
        Make a copy of this node, but not its children.
        You may pass in a Document pointer that will be
        the owner of the new Node. If the 'document' is
        null, then the node returned will be allocated
        from the current Document. (this->GetDocument())

        Note: if called on a XMLDocument, this will return null.
    */
    virtual XMLNode* ShallowClone( XMLDocument* document ) const = 0;

    /**
                Make a copy of this node and all its children.

                If the 'target' is null, then the nodes will
                be allocated in the current document. If 'target'
        is specified, the memory will be allocated is the
        specified XMLDocument.

                NOTE: This is probably not the correct tool to
                copy a document, since XMLDocuments can have multiple
                top level XMLNodes. You probably want to use
        XMLDocument::DeepCopy()
        */
    XMLNode* DeepClone( XMLDocument* target ) const;

    /**
        Test if 2 nodes are the same, but don't test children.
        The 2 nodes do not need to be in the same Document.

        Note: if called on a XMLDocument, this will return false.
    */
    virtual bool ShallowEqual( const XMLNode* compare ) const = 0;

    /** Accept a hierarchical visit of the nodes in the TinyXML-2 DOM. Every node in the
        XML tree will be conditionally visited and the host will be called back
        via the XMLVisitor interface.

        This is essentially a SAX interface for TinyXML-2. (Note however it doesn't re-parse
        the XML for the callbacks, so the performance of TinyXML-2 is unchanged by using this
        interface versus any other.)

        The interface has been based on ideas from:

        - http://www.saxproject.org/
        - http://c2.com/cgi/wiki?HierarchicalVisitorPattern

        Which are both good references for "visiting".

        An example of using Accept():
        @verbatim
        XMLPrinter printer;
        tinyxmlDoc.Accept( &printer );
        const char* xmlcstr = printer.CStr();
        @endverbatim
    */
    virtual bool Accept( XMLVisitor* visitor ) const = 0;

    /**
                Set user data into the XMLNode. TinyXML-2 in
                no way processes or interprets user data.
                It is initially 0.
        */
    void SetUserData(void* userData)	{ _userData = userData; }

    /**
                Get user data set into the XMLNode. TinyXML-2 in
                no way processes or interprets user data.
                It is initially 0.
        */
    void* GetUserData() const			{ return _userData; }

protected:
    explicit XMLNode( XMLDocument* const doc);
    virtual ~XMLNode();

    virtual char_t* ParseDeep( char_t* p, StrPair* const parentEndTag, int32_t* const curLineNumPtr );

    int32_t  GetParseLineNum() const { return _parseLineNum; }

    XMLDocument*	_document;
    mutable StrPair	_value;

private:
    XMLNode*		_parent;

    XMLNode*		_firstChild;
    XMLNode*		_lastChild;

    XMLNode*		_prev;
    XMLNode*		_next;

    int32_t         _parseLineNum;

    void*			_userData;

private:
    XMLNode( const XMLNode& ) = delete;	// not supported
    XMLNode& operator=( const XMLNode& ) = delete;	// not supported

    MemPool*		_memPool;
    void Unlink( XMLNode* child );
    static void DeleteNode( XMLNode* node );
    void InsertChildPreamble( XMLNode* insertThis ) const;
    const XMLElement* ToElementWithName( const char_t* const name ) const;
};


/** XML text.

        Note that a text node can have child element nodes, for example:
        @verbatim
        <root>This is <b>bold</b></root>
        @endverbatim

        A text node can have 2 ways to output the next. "normal" output
        and CDATA. It will default to the mode it was parsed from the XML file and
        you generally want to leave it alone, but you can change the output mode with
        SetCData() and query it with CData().
*/
class TINYXML2_LIB XMLText : public XMLNode
{
    friend class XMLDocument;

public:
    virtual bool Accept( XMLVisitor* visitor ) const;

    virtual XMLText* ToText()
    {
        return this;
    }
    virtual const XMLText* ToText() const
    {
        return this;
    }

    /// Declare whether this should be CDATA or standard text.
    void SetCData( bool isCData )
    {
        _isCData = isCData;
    }
    /// Returns true if this is a CDATA text element.
    bool CData() const
    {
        return _isCData;
    }

    virtual XMLNode* ShallowClone( XMLDocument* document ) const;
    virtual bool ShallowEqual( const XMLNode* compare ) const;

protected:
    explicit XMLText( XMLDocument* doc ) : XMLNode( doc ), _isCData( false ) {}
    virtual ~XMLText() = default;

    virtual char_t* ParseDeep( char_t* p, StrPair* parentEndTag, int32_t* curLineNumPtr );

private:
    XMLText( const XMLText& ) = delete;	// not supported
    XMLText& operator=( const XMLText& ) = delete;	// not supported

    bool _isCData;
};


/** An XML Comment. */
class TINYXML2_LIB XMLComment : public XMLNode
{
    friend class XMLDocument;

public:
    virtual XMLComment*	ToComment()
    {
        return this;
    }
    virtual const XMLComment* ToComment() const
    {
        return this;
    }

    virtual bool Accept( XMLVisitor* visitor ) const;

    virtual XMLNode* ShallowClone( XMLDocument* document ) const;
    virtual bool ShallowEqual( const XMLNode* compare ) const;

protected:
    explicit XMLComment( XMLDocument* doc );
    virtual ~XMLComment();

    virtual char_t* ParseDeep( char_t* p, StrPair* parentEndTag, int32_t* curLineNumPtr);

private:
    XMLComment( const XMLComment& ) = delete;	// not supported
    XMLComment& operator=( const XMLComment& ) = delete;	// not supported
};


/** In correct XML the declaration is the first entry in the file.
        @verbatim
                <?xml version="1.0" standalone="yes"?>
        @endverbatim

        TinyXML-2 will happily read or write files without a declaration,
        however.

        The text of the declaration isn't interpreted. It is parsed
        and written as a string.
*/
class TINYXML2_LIB XMLDeclaration : public XMLNode
{
    friend class XMLDocument;

public:
    virtual XMLDeclaration*	ToDeclaration()
    {
        return this;
    }
    virtual const XMLDeclaration* ToDeclaration() const
    {
        return this;
    }

    virtual bool Accept( XMLVisitor* visitor ) const;

    virtual XMLNode* ShallowClone( XMLDocument* document ) const;
    virtual bool ShallowEqual( const XMLNode* compare ) const;

protected:
    explicit XMLDeclaration( XMLDocument* doc );
    virtual ~XMLDeclaration();

    virtual char_t* ParseDeep( char_t* p, StrPair* parentEndTag, int32_t* curLineNumPtr );

private:
    XMLDeclaration( const XMLDeclaration& ) = delete;	// not supported
    XMLDeclaration& operator=( const XMLDeclaration& ) = delete;	// not supported
};


/** Any tag that TinyXML-2 doesn't recognize is saved as an
        unknown. It is a tag of text, but should not be modified.
        It will be written back to the XML, unchanged, when the file
        is saved.

        DTD tags get thrown into XMLUnknowns.
*/
class TINYXML2_LIB XMLUnknown : public XMLNode
{
    friend class XMLDocument;

public:
    virtual XMLUnknown*	ToUnknown()
    {
        return this;
    }
    virtual const XMLUnknown* ToUnknown() const
    {
        return this;
    }

    virtual bool Accept( XMLVisitor* visitor ) const;

    virtual XMLNode* ShallowClone( XMLDocument* document ) const;
    virtual bool ShallowEqual( const XMLNode* compare ) const;

protected:
    explicit XMLUnknown( XMLDocument* doc );
    virtual ~XMLUnknown();

    virtual char_t* ParseDeep( char_t* p, StrPair* parentEndTag, int32_t* curLineNumPtr );

private:
    XMLUnknown( const XMLUnknown& ) = delete;	// not supported
    XMLUnknown& operator=( const XMLUnknown& ) = delete;	// not supported
};



/** An attribute is a name-value pair. Elements have an arbitrary
        number of attributes, each with a unique name.

        @note The attributes are not XMLNodes. You may only query the
        Next() attribute in a list.
*/
class TINYXML2_LIB XMLAttribute
{
    friend class XMLElement;

public:
    /// The name of the attribute.
    const char_t* Name() const;

    /// The value of the attribute.
    const char_t* Value() const;

    /// Gets the line number the attribute is in, if the document was parsed from a file.
    int32_t GetLineNum() const
    {
        return _parseLineNum;
    }

    /// The next attribute in the list.
    const XMLAttribute* Next() const
    {
        return _next;
    }

    /** QueryIntValue interprets the attribute as an integer, and returns the value
    in the provided parameter. The function will return XML_SUCCESS on success,
    and XML_WRONG_ATTRIBUTE_TYPE if the conversion is not successful.
*/
    XMLError QueryIntValue( int32_t* value ) const;
    /// See QueryIntValue
    XMLError QueryUnsignedValue( uint32_t* value ) const;
    /// See QueryIntValue
    XMLError QueryInt64Value(int64_t* value) const;
    /// See QueryIntValue
    XMLError QueryUnsigned64Value(uint64_t* value) const;
    /// See QueryIntValue
    XMLError QueryBoolValue( bool* value ) const;
    /// See QueryIntValue
    XMLError QueryDoubleValue( float64_t* value ) const;
    /// See QueryIntValue
    XMLError QueryFloatValue( float32_t* value ) const;

    /** IntValue interprets the attribute as an integer, and returns the value.
        If the value isn't an integer, 0 will be returned. There is no error checking;
        use QueryIntValue() if you need error checking.
    */
    int32_t	IntValue() const
    {
        int32_t i = 0;
        (void)QueryIntValue(&i);
        return i;
    }

    int64_t Int64Value() const
    {
        int64_t i = 0;
        (void)QueryInt64Value(&i);
        return i;
    }

    uint64_t Unsigned64Value() const
    {
        uint64_t i = 0;
        (void)QueryUnsigned64Value(&i);
        return i;
    }

    /// Query as an unsigned integer. See IntValue()
    uint32_t UnsignedValue() const
    {
        uint32_t i = 0;
        (void)QueryUnsignedValue( &i );
        return i;
    }
    /// Query as a boolean. See IntValue()
    bool	 BoolValue() const
    {
        bool b = false;
        (void)QueryBoolValue( &b );
        return b;
    }
    /// Query as a double. See IntValue()
    float64_t 	 DoubleValue() const
    {
        float64_t d = 0.0;
        (void)QueryDoubleValue( &d );
        return d;
    }
    /// Query as a float. See IntValue()
    float32_t	 FloatValue() const
    {
        float32_t f = 0.0;
        (void)(QueryFloatValue)( &f );
        return f;
    }

    /// Set the attribute to a string value.
    void SetAttribute( const char_t* v );
    /// Set the attribute to value.
    void SetAttribute( int32_t v );
    /// Set the attribute to value.
    void SetAttribute( uint32_t v );
    /// Set the attribute to value.
    void SetAttribute(int64_t v);
    /// Set the attribute to value.
    void SetAttribute(uint64_t v);
    /// Set the attribute to value.
    void SetAttribute( bool v );
    /// Set the attribute to value.
    void SetAttribute( float64_t v );
    /// Set the attribute to value.
    void SetAttribute( float32_t v );

private:
    enum { BUF_SIZE = 200 };

    XMLAttribute() : _name(), _value(),_parseLineNum( 0 ), _next(nullptr), _memPool(nullptr) {}
    virtual ~XMLAttribute()	= default;

    XMLAttribute( const XMLAttribute& ) = delete;	// not supported
    void operator=( const XMLAttribute& ) = delete;	// not supported

    void SetName( const char_t* n );

    char_t* ParseDeep( char_t* p, bool processEntities, int32_t* curLineNumPtr );

    mutable StrPair _name;
    mutable StrPair _value;
    int32_t             _parseLineNum;
    XMLAttribute*   _next;
    MemPool*        _memPool;
};


/** The element is a container class. It has a value, the element name,
        and can contain other elements, text, comments, and unknowns.
        Elements also contain an arbitrary number of attributes.
*/
class TINYXML2_LIB XMLElement : public XMLNode
{
    friend class XMLDocument;

private:
    XMLAttribute* FindOrCreateAttribute( const char_t* name );

public:
    /// Get the name of an element (which is the Value() of the node.)
    const char_t* Name() const
    {
        return Value();
    }
    /// Set the name of the element.
    void SetName( const char_t* str, bool staticMem=false )
    {
        SetValue( str, staticMem );
    }

    virtual XMLElement* ToElement()
    {
        return this;
    }
    virtual const XMLElement* ToElement() const
    {
        return this;
    }

    virtual bool Accept( XMLVisitor* visitor ) const;

    /** Given an attribute name, Attribute() returns the value
        for the attribute of that name, or null if none
        exists. For example:

        @verbatim
        const char* value = ele->Attribute( "foo" );
        @endverbatim

        The 'value' parameter is normally null. However, if specified,
        the attribute will only be returned if the 'name' and 'value'
        match. This allow you to write code:

        @verbatim
        if ( ele->Attribute( "foo", "bar" ) ) callFooIsBar();
        @endverbatim

        rather than:
        @verbatim
        if ( ele->Attribute( "foo" ) ) {
                if ( strcmp( ele->Attribute( "foo" ), "bar" ) == 0 ) callFooIsBar();
        }
        @endverbatim
    */
    const char_t* Attribute( const char_t* name, const char_t* value=nullptr) const;

    /** Given an attribute name, IntAttribute() returns the value
        of the attribute interpreted as an integer. The default
        value will be returned if the attribute isn't present,
        or if there is an error. (For a method with error
        checking, see QueryIntAttribute()).
    */
    int32_t IntAttribute(const char_t* name, int32_t defaultValue = 0) const;
    /// See IntAttribute()
    uint32_t UnsignedAttribute(const char_t* name, uint32_t defaultValue = 0) const;
    /// See IntAttribute()
    int64_t Int64Attribute(const char_t* name, int64_t defaultValue = 0) const;
    /// See IntAttribute()
    uint64_t Unsigned64Attribute(const char_t* name, uint64_t defaultValue = 0) const;
    /// See IntAttribute()
    bool BoolAttribute(const char_t* name, bool defaultValue = false) const;
    /// See IntAttribute()
    float64_t DoubleAttribute(const char_t* name, float64_t defaultValue = 0.0) const;
    /// See IntAttribute()
    float32_t FloatAttribute(const char_t* name, float32_t defaultValue = 0.0) const;

    /// Query a specific attribute in the list.
    const XMLAttribute* FindAttribute( const char_t* name ) const;

    /** Given an attribute name, QueryIntAttribute() returns
        XML_SUCCESS, XML_WRONG_ATTRIBUTE_TYPE if the conversion
        can't be performed, or XML_NO_ATTRIBUTE if the attribute
        doesn't exist. If successful, the result of the conversion
        will be written to 'value'. If not successful, nothing will
        be written to 'value'. This allows you to provide default
        value:

        @verbatim
        int value = 10;
        QueryIntAttribute( "foo", &value );		// if "foo" isn't found, value will still be 10
        @endverbatim
    */
    XMLError QueryIntAttribute( const char_t* name, int32_t* value ) const
    {
        const XMLAttribute* a = (FindAttribute)( name );
        const XMLError error = ( a == nullptr ) ? XML_NO_ATTRIBUTE : a->QueryIntValue( value );
        return error;
    }

    /// See QueryIntAttribute()
    XMLError QueryUnsignedAttribute( const char_t* name, uint32_t* value ) const
    {
        const XMLAttribute* a = (FindAttribute)( name );
        const XMLError error = ( a == nullptr ) ? XML_NO_ATTRIBUTE : a->QueryUnsignedValue(value);
        return error;
    }

    /// See QueryIntAttribute()
    XMLError QueryInt64Attribute(const char_t* name, int64_t* value) const
    {
        const XMLAttribute* a = (FindAttribute)(name);
        const XMLError error = ( a == nullptr ) ? XML_NO_ATTRIBUTE : a->QueryInt64Value(value);
        return error;
    }

    /// See QueryIntAttribute()
    XMLError QueryUnsigned64Attribute(const char_t* name, uint64_t* value) const
    {
        const XMLAttribute* a = (FindAttribute)(name);
        const XMLError error = ( a == nullptr ) ? XML_NO_ATTRIBUTE : a->QueryUnsigned64Value(value);
        return error;
    }

    /// See QueryIntAttribute()
    XMLError QueryBoolAttribute( const char_t* name, bool* value ) const
    {
        const XMLAttribute* a = (FindAttribute)( name );
        const XMLError error = ( a == nullptr ) ? XML_NO_ATTRIBUTE : a->QueryBoolValue(value);
        return error;
    }
    /// See QueryIntAttribute()
    XMLError QueryDoubleAttribute( const char_t* name, float64_t* value ) const
    {
        const XMLAttribute* a = (FindAttribute)( name );
        const XMLError error = ( a == nullptr ) ? XML_NO_ATTRIBUTE : a->QueryDoubleValue(value);
        return error;
    }
    /// See QueryIntAttribute()
    XMLError QueryFloatAttribute( const char_t* name, float32_t* value ) const
    {
        const XMLAttribute* a = (FindAttribute)( name );
        const XMLError error = ( a == nullptr ) ? XML_NO_ATTRIBUTE : a->QueryFloatValue(value);
        return error;
    }

    /// See QueryIntAttribute()
    XMLError QueryStringAttribute(const char_t* name, const char_t** value) const
    {
        const XMLAttribute* a = (FindAttribute)(name);
        if ( a == nullptr ) {
            return XML_NO_ATTRIBUTE;
        }
        *value = a->Value();
        return XML_SUCCESS;
    }



    /** Given an attribute name, QueryAttribute() returns
        XML_SUCCESS, XML_WRONG_ATTRIBUTE_TYPE if the conversion
        can't be performed, or XML_NO_ATTRIBUTE if the attribute
        doesn't exist. It is overloaded for the primitive types,
                and is a generally more convenient replacement of
                QueryIntAttribute() and related functions.

                If successful, the result of the conversion
        will be written to 'value'. If not successful, nothing will
        be written to 'value'. This allows you to provide default
        value:

        @verbatim
        int value = 10;
        QueryAttribute( "foo", &value );		// if "foo" isn't found, value will still be 10
        @endverbatim
    */
    XMLError QueryAttribute( const char_t* name, int32_t* value ) const
    {
        return QueryIntAttribute( name, value );
    }

    XMLError QueryAttribute( const char_t* name, uint32_t* value ) const
    {
        return QueryUnsignedAttribute( name, value );
    }

    XMLError QueryAttribute(const char_t* name, int64_t* value) const
    {
        return QueryInt64Attribute(name, value);
    }

    XMLError QueryAttribute(const char_t* name, uint64_t* value) const
    {
        return QueryUnsigned64Attribute(name, value);
    }

    XMLError QueryAttribute( const char_t* name, bool* value ) const
    {
        return QueryBoolAttribute( name, value );
    }

    XMLError QueryAttribute( const char_t* name, float64_t* value ) const
    {
        return QueryDoubleAttribute( name, value );
    }

    XMLError QueryAttribute( const char_t* name, float32_t* value ) const
    {
        return QueryFloatAttribute( name, value );
    }

    /// Sets the named attribute to value.
    void SetAttribute( const char_t* name, const char_t* value )
    {
        XMLAttribute* a = (FindOrCreateAttribute)( name );
        a->SetAttribute( value );
    }
    /// Sets the named attribute to value.
    void SetAttribute( const char_t* name, int32_t value )
    {
        XMLAttribute* a = (FindOrCreateAttribute)( name );
        a->SetAttribute( value );
    }
    /// Sets the named attribute to value.
    void SetAttribute( const char_t* name, uint32_t value )
    {
        XMLAttribute* a = (FindOrCreateAttribute)( name );
        a->SetAttribute( value );
    }
    /// Sets the named attribute to value.
    void SetAttribute(const char_t* name, int64_t value)
    {
        XMLAttribute* a = (FindOrCreateAttribute)(name);
        a->SetAttribute(value);
    }
    /// Sets the named attribute to value.
    void SetAttribute(const char_t* name, uint64_t value)
    {
        XMLAttribute* a = (FindOrCreateAttribute)(name);
        a->SetAttribute(value);
    }
    /// Sets the named attribute to value.
    void SetAttribute( const char_t* name, bool value )
    {
        XMLAttribute* a = (FindOrCreateAttribute)( name );
        a->SetAttribute( value );
    }
    /// Sets the named attribute to value.
    void SetAttribute( const char_t* name, float64_t value )
    {
        XMLAttribute* a = (FindOrCreateAttribute)( name );
        a->SetAttribute( value );
    }
    /// Sets the named attribute to value.
    void SetAttribute( const char_t* name, float32_t value )
    {
        XMLAttribute* a = (FindOrCreateAttribute)( name );
        a->SetAttribute( value );
    }

    /**
        Delete an attribute.
    */
    void DeleteAttribute( const char_t* name );

    /// Return the first attribute in the list.
    const XMLAttribute* FirstAttribute() const
    {
        return _rootAttribute;
    }

    /** Convenience function for easy access to the text inside an element. Although easy
        and concise, GetText() is limited compared to getting the XMLText child
        and accessing it directly.

        If the first child of 'this' is a XMLText, the GetText()
        returns the character string of the Text node, else null is returned.

        This is a convenient method for getting the text of simple contained text:
        @verbatim
        <foo>This is text</foo>
                const char* str = fooElement->GetText();
        @endverbatim

        'str' will be a pointer to "This is text".

        Note that this function can be misleading. If the element foo was created from
        this XML:
        @verbatim
                <foo><b>This is text</b></foo>
        @endverbatim

        then the value of str would be null. The first child node isn't a text node, it is
        another element. From this XML:
        @verbatim
                <foo>This is <b>text</b></foo>
        @endverbatim
        GetText() will return "This is ".
    */
    const char_t* GetText() const;

    /** Convenience function for easy access to the text inside an element. Although easy
        and concise, SetText() is limited compared to creating an XMLText child
        and mutating it directly.

        If the first child of 'this' is a XMLText, SetText() sets its value to
                the given string, otherwise it will create a first child that is an XMLText.

        This is a convenient method for setting the text of simple contained text:
        @verbatim
        <foo>This is text</foo>
                fooElement->SetText( "Hullaballoo!" );
        <foo>Hullaballoo!</foo>
                @endverbatim

        Note that this function can be misleading. If the element foo was created from
        this XML:
        @verbatim
                <foo><b>This is text</b></foo>
        @endverbatim

        then it will not change "This is text", but rather prefix it with a text element:
        @verbatim
                <foo>Hullaballoo!<b>This is text</b></foo>
        @endverbatim

                For this XML:
        @verbatim
                <foo />
        @endverbatim
        SetText() will generate
        @verbatim
                <foo>Hullaballoo!</foo>
        @endverbatim
    */
    void SetText( const char_t* inText );
    /// Convenience method for setting text inside an element. See SetText() for important limitations.
    void SetText( int32_t v );
    /// Convenience method for setting text inside an element. See SetText() for important limitations.
    void SetText( uint32_t v );
    /// Convenience method for setting text inside an element. See SetText() for important limitations.
    void SetText(int64_t v);
    /// Convenience method for setting text inside an element. See SetText() for important limitations.
    void SetText(uint64_t v);
    /// Convenience method for setting text inside an element. See SetText() for important limitations.
    void SetText( bool v );
    /// Convenience method for setting text inside an element. See SetText() for important limitations.
    void SetText( float64_t v );
    /// Convenience method for setting text inside an element. See SetText() for important limitations.
    void SetText( float32_t v );

    /**
        Convenience method to query the value of a child text node. This is probably best
        shown by example. Given you have a document is this form:
        @verbatim
                <point>
                        <x>1</x>
                        <y>1.4</y>
                </point>
        @endverbatim

        The QueryIntText() and similar functions provide a safe and easier way to get to the
        "value" of x and y.

        @verbatim
                int x = 0;
                float y = 0;	// types of x and y are contrived for example
                const XMLElement* xElement = pointElement->FirstChildElement( "x" );
                const XMLElement* yElement = pointElement->FirstChildElement( "y" );
                xElement->QueryIntText( &x );
                yElement->QueryFloatText( &y );
        @endverbatim

        @returns XML_SUCCESS (0) on success, XML_CAN_NOT_CONVERT_TEXT if the text cannot be converted
                         to the requested type, and XML_NO_TEXT_NODE if there is no child text to query.

    */
    XMLError QueryIntText( int32_t* ival ) const;
    /// See QueryIntText()
    XMLError QueryUnsignedText( uint32_t* uval ) const;
    /// See QueryIntText()
    XMLError QueryInt64Text(int64_t* ival) const;
    /// See QueryIntText()
    XMLError QueryUnsigned64Text(uint64_t* ival) const;
    /// See QueryIntText()
    XMLError QueryBoolText( bool* bval ) const;
    /// See QueryIntText()
    XMLError QueryDoubleText( float64_t* dval ) const;
    /// See QueryIntText()
    XMLError QueryFloatText( float32_t* fval ) const;

    int32_t IntText(int32_t defaultValue = 0) const;
    /// See QueryIntText()
    uint32_t UnsignedText(uint32_t defaultValue = 0) const;
    /// See QueryIntText()
    int64_t Int64Text(int64_t defaultValue = 0) const;
    /// See QueryIntText()
    uint64_t Unsigned64Text(uint64_t defaultValue = 0) const;
    /// See QueryIntText()
    bool BoolText(bool defaultValue = false) const;
    /// See QueryIntText()
    float64_t DoubleText(float64_t defaultValue = 0.0) const;
    /// See QueryIntText()
    float32_t FloatText(float32_t defaultValue = 0.0) const;

    /**
        Convenience method to create a new XMLElement and add it as last (right)
        child of this node. Returns the created and inserted element.
    */
    XMLElement* InsertNewChildElement(const char_t* const name);
    /// See InsertNewChildElement()
    XMLComment* InsertNewComment(const char_t* comment);
    /// See InsertNewChildElement()
    XMLText* InsertNewText(const char_t* text);
    /// See InsertNewChildElement()
    XMLDeclaration* InsertNewDeclaration(const char_t* text);
    /// See InsertNewChildElement()
    XMLUnknown* InsertNewUnknown(const char_t* text);

    // internal:
    enum ElementClosingType
    {
        OPEN,		// <foo>
        CLOSED,		// <foo/>
        CLOSING		// </foo>
    };
    ElementClosingType ClosingType() const
    {
        return _closingType;
    }

    virtual XMLNode* ShallowClone( XMLDocument* document ) const;
    virtual bool ShallowEqual( const XMLNode* compare ) const;

protected:
    virtual char_t* ParseDeep( char_t* p, StrPair* parentEndTag, int32_t* curLineNumPtr );

private:
    explicit XMLElement( XMLDocument* doc );
    virtual ~XMLElement();

    XMLElement( const XMLElement& ) = delete;	// not supported
    void operator=( const XMLElement& ) = delete;	// not supported

    //    XMLAttribute* FindOrCreateAttribute( const char_t* name );
    char_t* ParseAttributes( char_t* p, int32_t* curLineNumPtr );
    static void DeleteAttribute( XMLAttribute* attribute );
    XMLAttribute* CreateAttribute();

    enum { BUF_SIZE = 200 };

    ElementClosingType _closingType;
    // The attribute list is ordered; there is no 'lastAttribute'
    // because the list needs to be scanned for dupes before adding
    // a new attribute.
    XMLAttribute* _rootAttribute;
};


enum Whitespace
{
    PRESERVE_WHITESPACE,
    COLLAPSE_WHITESPACE
};

// This is likely overengineered template art to have a check that unsigned long value incremented
// by one still fits into size_t. If size_t type is larger than unsigned long type
// (x86_64-w64-mingw32 target) then the check is redundant and gcc and clang emit
// -Wtype-limits warning. This piece makes the compiler select code with a check when a check
// is useful and code with no check when a check is redundant depending on how size_t and unsigned long
// types sizes relate to each other.
template
<bool = (sizeof(uint64_t) >= sizeof(size_t))>
struct LongFitsIntoSizeTMinusOne
{
    static bool Fits( uint64_t value )
    {
        return value < static_cast<size_t>(-1);
    }
};

template <>
struct LongFitsIntoSizeTMinusOne<false>
{
    static bool Fits( uint64_t )
    {
        return true;
    }
};

/** A Document binds together all the functionality.
        It can be saved, loaded, and printed to the screen.
        All Nodes are connected and allocated to a Document.
        If the Document is deleted, all its Nodes are also deleted.
*/
class TINYXML2_LIB XMLDocument : public XMLNode
{
    friend class XMLElement;
    // Gives access to SetError and Push/PopDepth, but over-access for everything else.
    // Wishing C++ had "internal" scope.
    friend class XMLNode;
    friend class XMLText;
    friend class XMLComment;
    friend class XMLDeclaration;
    friend class XMLUnknown;

private:
    void SetError( XMLError error, int32_t lineNum, const char_t* format, ... );

public:
    /// constructor
    explicit XMLDocument( bool processEntities = true, Whitespace whitespaceMode = PRESERVE_WHITESPACE );
    virtual ~XMLDocument();

    virtual XMLDocument* ToDocument()
    {
        TIXMLASSERT( this == _document );
        return this;
    }
    virtual const XMLDocument* ToDocument() const
    {
        TIXMLASSERT( this == _document );
        return this;
    }

    /**
        Parse an XML file from a character string.
        Returns XML_SUCCESS (0) on success, or
        an errorID.

        You may optionally pass in the 'nBytes', which is
        the number of bytes which will be parsed. If not
        specified, TinyXML-2 will assume 'xml' points to a
        null terminated string.
    */
    XMLError Parse( const char_t* p, size_t len=static_cast<size_t>(-1) );

    /**
        Load an XML file from disk.
        Returns XML_SUCCESS (0) on success, or
        an errorID.
    */
    XMLError LoadFile( const char_t* filename );

    /**
        Load an XML file from disk. You are responsible
        for providing and closing the FILE*.

        NOTE: The file should be opened as binary ("rb")
        not text in order for TinyXML-2 to correctly
        do newline normalization.

        Returns XML_SUCCESS (0) on success, or
        an errorID.
    */
    XMLError LoadFile( FILE* );

    /**
        Save the XML file to disk.
        Returns XML_SUCCESS (0) on success, or
        an errorID.
    */
    XMLError SaveFile( const char_t* filename, bool compact = false );

    /**
        Save the XML file to disk. You are responsible
        for providing and closing the FILE*.

        Returns XML_SUCCESS (0) on success, or
        an errorID.
    */
    XMLError SaveFile( const FILE* const fp, bool compact = false );

    bool ProcessEntities() const
    {
        return _processEntities;
    }

    Whitespace WhitespaceMode() const
    {
        return _whitespaceMode;
    }

    /**
        Returns true if this document has a leading Byte Order Mark of UTF8.
    */
    bool HasBOM() const
    {
        return _writeBOM;
    }
    /** Sets whether to write the BOM when writing the file.
    */
    void SetBOM( bool useBOM )
    {
        _writeBOM = useBOM;
    }

    /** Return the root element of DOM. Equivalent to FirstChildElement().
        To get the first node, use FirstChild().
    */
    XMLElement* RootElement()
    {

        return FirstChildElement();
    }
    const XMLElement* RootElement() const
    {

        return FirstChildElement();
    }

    /** Print the Document. If the Printer is not provided, it will
        print to stdout. If you provide Printer, this can print to a file:
        @verbatim
        XMLPrinter printer( fp );
        doc.Print( &printer );
        @endverbatim

        Or you can use a printer to print to memory:
        @verbatim
        XMLPrinter printer;
        doc.Print( &printer );
        // printer.CStr() has a const char* to the XML
        @endverbatim
    */
    void Print( XMLPrinter* streamer=nullptr) const;
    virtual bool Accept( XMLVisitor* visitor ) const;

    /**
        Create a new Element associated with
        this Document. The memory for the Element
        is managed by the Document.
    */
    XMLElement* NewElement( const char_t* name );
    /**
        Create a new Comment associated with
        this Document. The memory for the Comment
        is managed by the Document.
    */
    XMLComment* NewComment( const char_t* str );
    /**
        Create a new Text associated with
        this Document. The memory for the Text
        is managed by the Document.
    */
    XMLText* NewText( const char_t* str );
    /**
        Create a new Declaration associated with
        this Document. The memory for the object
        is managed by the Document.

        If the 'text' param is null, the standard
        declaration is used.:
        @verbatim
                <?xml version="1.0" encoding="UTF-8"?>
        @endverbatim
    */
    XMLDeclaration* NewDeclaration( const char_t* str=nullptr);
    /**
        Create a new Unknown associated with
        this Document. The memory for the object
        is managed by the Document.
    */
    XMLUnknown* NewUnknown( const char_t* str );

    /**
        Delete a node associated with this document.
        It will be unlinked from the DOM.
    */
    void DeleteNode( XMLNode* node );

    void ClearError()
    {
        (SetError)(XML_SUCCESS, 0, nullptr);
    }

    /// Return true if there was an error parsing the document.
    bool Error() const
    {
        return _errorID != XML_SUCCESS;
    }
    /// Return the errorID.
    XMLError  ErrorID() const
    {
        return _errorID;
    }

    const char_t* ErrorName() const;
    static const char_t* ErrorIDToName(XMLError errorID);

    /** Returns a "long form" error description. A hopefully helpful
        diagnostic with location, line number, and/or additional info.
    */
    const char_t* ErrorStr() const;

    /// A (trivial) utility function that prints the ErrorStr() to stdout.
    void PrintError() const;

    /// Return the line where the error occurred, or zero if unknown.
    int32_t ErrorLineNum() const
    {
        return _errorLineNum;
    }

    /// Clear the document, resetting it to the initial state.
    void Clear();

    /**
                Copies this document to a target document.
                The target will be completely cleared before the copy.
                If you want to copy a sub-tree, see XMLNode::DeepClone().

                NOTE: that the 'target' must be non-null.
        */
    void DeepCopy(XMLDocument* target) const;

    // internal
    char_t* Identify( char_t* p, XMLNode** node );

    // internal
    void MarkInUse( const XMLNode* const);

    virtual XMLNode* ShallowClone( XMLDocument* /*document*/ ) const
    {
        return nullptr;
    }
    virtual bool ShallowEqual( const XMLNode* /*compare*/ ) const
    {
        return false;
    }

private:
    XMLDocument( const XMLDocument& ) = delete;	// not supported
    void operator=( const XMLDocument& ) = delete;	// not supported

    bool			_writeBOM;
    bool			_processEntities;
    XMLError		_errorID;
    Whitespace		_whitespaceMode;
    mutable StrPair	_errorStr;
    int32_t         _errorLineNum;
    char_t*			_charBuffer;
    int32_t			_parseCurLineNum;
    int32_t			_parsingDepth;
    // Memory tracking does add some overhead.
    // However, the code assumes that you don't
    // have a bunch of unlinked nodes around.
    // Therefore it takes less memory to track
    // in the document vs. a linked list in the XMLNode,
    // and the performance is the same.
    DynArray<XMLNode*, 10> _unlinked;

    MemPoolT< sizeof(XMLElement) >	 _elementPool;
    MemPoolT< sizeof(XMLAttribute) > _attributePool;
    MemPoolT< sizeof(XMLText) >		 _textPool;
    MemPoolT< sizeof(XMLComment) >	 _commentPool;

    static const char_t* _errorNames[XML_ERROR_COUNT];

    void Parse();

    void PushDepth();
    void PopDepth();

    // Something of an obvious security hole, once it was discovered.
    // Either an ill-formed XML or an excessively deep one can overflow
    // the stack. Track stack depth, and error out if needed.
    class DepthTracker
    {
    public:
        explicit DepthTracker(XMLDocument * document)
        {
            this->_document = document;
            document->PushDepth();
        }

        ~DepthTracker()
        {
            _document->PopDepth();
        }

    private:
        XMLDocument * _document;
    };

    template<class NodeType, int32_t PoolElementSize>
    NodeType* CreateUnlinkedNode( MemPoolT<PoolElementSize>& pool );
};

template<class NodeType, int32_t PoolElementSize>
inline NodeType* XMLDocument::CreateUnlinkedNode( MemPoolT<PoolElementSize>& pool )
{
    TIXMLASSERT( sizeof( NodeType ) == PoolElementSize );
    TIXMLASSERT( sizeof( NodeType ) == pool.ItemSize() );

    NodeType* returnNode = new (pool.Alloc()) NodeType( this );
    if (returnNode != nullptr)
    {
        TIXMLASSERT(returnNode);
        returnNode->_memPool = &pool;
        _unlinked.Push(returnNode);
    }

    return returnNode;
}

/**
        A XMLHandle is a class that wraps a node pointer with null checks; this is
        an incredibly useful thing. Note that XMLHandle is not part of the TinyXML-2
        DOM structure. It is a separate utility class.

        Take an example:
        @verbatim
        <Document>
                <Element attributeA = "valueA">
                        <Child attributeB = "value1" />
                        <Child attributeB = "value2" />
                </Element>
        </Document>
        @endverbatim

        Assuming you want the value of "attributeB" in the 2nd "Child" element, it's very
        easy to write a *lot* of code that looks like:

        @verbatim
        XMLElement* root = document.FirstChildElement( "Document" );
        if ( root )
        {
                XMLElement* element = root->FirstChildElement( "Element" );
                if ( element )
                {
                        XMLElement* child = element->FirstChildElement( "Child" );
                        if ( child )
                        {
                                XMLElement* child2 = child->NextSiblingElement( "Child" );
                                if ( child2 )
                                {
                                        // Finally do something useful.
        @endverbatim

        And that doesn't even cover "else" cases. XMLHandle addresses the verbosity
        of such code. A XMLHandle checks for null pointers so it is perfectly safe
        and correct to use:

        @verbatim
        XMLHandle docHandle( &document );
        XMLElement* child2 = docHandle.FirstChildElement( "Document" ).FirstChildElement( "Element" ).FirstChildElement().NextSiblingElement();
        if ( child2 )
        {
                // do something useful
        @endverbatim

        Which is MUCH more concise and useful.

        It is also safe to copy handles - internally they are nothing more than node pointers.
        @verbatim
        XMLHandle handleCopy = handle;
        @endverbatim

        See also XMLConstHandle, which is the same as XMLHandle, but operates on const objects.
*/
class TINYXML2_LIB XMLHandle
{
public:
    /// Create a handle from any node (at any depth of the tree.) This can be a null pointer.
    explicit XMLHandle( XMLNode* const node ) : _node( node )
    {}
    /// Create a handle from a node.
    explicit XMLHandle( const XMLNode& node ) : _node( const_cast<XMLNode*>(&node) )
    {}

    /// Copy constructor
    XMLHandle( const XMLHandle& ref ) : _node( ref._node )
    {}
    /// Assignment
    XMLHandle& operator=( const XMLHandle& ref )
    {
        _node = ref._node;
        return *this;
    }

    /// Get the first child of this handle.
    XMLHandle FirstChild() 	const
    {
        return XMLHandle( (_node != nullptr) ? _node->FirstChild() : nullptr);
    }
    /// Get the first child element of this handle.
    XMLHandle FirstChildElement( const char_t* name = nullptr)
    {
        return XMLHandle( (_node != nullptr) ? _node->FirstChildElement( name ) : nullptr);
    }
    /// Get the last child of this handle.
    XMLHandle LastChild()
    {
        return XMLHandle( (_node != nullptr) ? _node->LastChild() : nullptr);
    }
    /// Get the last child element of this handle.
    XMLHandle LastChildElement( const char_t* const name = nullptr)
    {
        return XMLHandle( (_node != nullptr) ? _node->LastChildElement( name ) : nullptr);
    }
    /// Get the previous sibling of this handle.
    XMLHandle PreviousSibling()
    {
        return XMLHandle( (_node != nullptr) ? _node->PreviousSibling() : nullptr);
    }
    /// Get the previous sibling element of this handle.
    XMLHandle PreviousSiblingElement( const char_t* name = nullptr)
    {
        return XMLHandle( (_node != nullptr) ? _node->PreviousSiblingElement( name ) : nullptr);
    }
    /// Get the next sibling of this handle.
    XMLHandle NextSibling()
    {
        return XMLHandle( (_node != nullptr) ? _node->NextSibling() : nullptr);
    }
    /// Get the next sibling element of this handle.
    XMLHandle NextSiblingElement( const char_t* name = nullptr)
    {
        return XMLHandle( (_node != nullptr) ? _node->NextSiblingElement( name ) : nullptr);
    }

    /// Safe cast to XMLNode. This can return null.
    const XMLNode* ToNode() const
    {
        return _node;
    }
    /// Safe cast to XMLElement. This can return null.
    XMLElement* ToElement()
    {
        return ( (_node != nullptr) ? _node->ToElement() : nullptr);
    }
    /// Safe cast to XMLText. This can return null.
    XMLText* ToText()
    {
        return ( (_node != nullptr) ? _node->ToText() : nullptr);
    }
    /// Safe cast to XMLUnknown. This can return null.
    XMLUnknown* ToUnknown()
    {
        return ( (_node != nullptr) ? _node->ToUnknown() : nullptr);
    }
    /// Safe cast to XMLDeclaration. This can return null.
    XMLDeclaration* ToDeclaration()
    {
        return ( (_node != nullptr) ? _node->ToDeclaration() : nullptr);
    }

private:
    XMLNode* _node;
};


/**
        A variant of the XMLHandle class for working with const XMLNodes and Documents. It is the
        same in all regards, except for the 'const' qualifiers. See XMLHandle for API.
*/
class TINYXML2_LIB XMLConstHandle
{
public:
    explicit XMLConstHandle( const XMLNode* node ) : _node( node )
    {}
    explicit XMLConstHandle( const XMLNode& node ) : _node( &node )
    {}

    XMLConstHandle( const XMLConstHandle& ref ) : _node( ref._node )
    {}
    XMLConstHandle& operator=( const XMLConstHandle& ref )
    {
        _node = ref._node;
        return *this;
    }

    XMLConstHandle FirstChild() const
    {
        return XMLConstHandle( (_node != nullptr) ? _node->FirstChild() : nullptr);
    }
    XMLConstHandle FirstChildElement( const char_t* name = nullptr) const
    {
        return XMLConstHandle( (_node != nullptr) ? _node->FirstChildElement( name ) : nullptr);
    }
    XMLConstHandle LastChild()	const
    {
        return XMLConstHandle( (_node != nullptr) ? _node->LastChild() : nullptr);
    }
    XMLConstHandle LastChildElement( const char_t* const name = nullptr) const
    {
        return XMLConstHandle( (_node != nullptr) ? _node->LastChildElement( name ) : nullptr);
    }
    XMLConstHandle PreviousSibling() const
    {
        return XMLConstHandle( (_node != nullptr) ? _node->PreviousSibling() : nullptr);
    }
    XMLConstHandle PreviousSiblingElement( const char_t* name = nullptr) const
    {
        return XMLConstHandle( (_node != nullptr) ? _node->PreviousSiblingElement( name ) : nullptr);
    }
    XMLConstHandle NextSibling() const
    {
        return XMLConstHandle( (_node != nullptr) ? _node->NextSibling() : nullptr);
    }
    XMLConstHandle NextSiblingElement( const char_t* name = nullptr) const
    {
        return XMLConstHandle( (_node != nullptr) ? _node->NextSiblingElement( name ) : nullptr);
    }


    const XMLNode* ToNode() const
    {
        return _node;
    }
    const XMLElement* ToElement() const
    {
        return ( (_node != nullptr) ? _node->ToElement() : nullptr);
    }
    const XMLText* ToText() const
    {
        return ( (_node != nullptr) ? _node->ToText() : nullptr);
    }
    const XMLUnknown* ToUnknown() const
    {
        return ( (_node != nullptr) ? _node->ToUnknown() : nullptr);
    }
    const XMLDeclaration* ToDeclaration() const
    {
        return ( (_node != nullptr) ? _node->ToDeclaration() : nullptr);
    }

private:
    const XMLNode* _node;
};


/**
        Printing functionality. The XMLPrinter gives you more
        options than the XMLDocument::Print() method.

        It can:
        -# Print to memory.
        -# Print to a file you provide.
        -# Print XML without a XMLDocument.

        Print to Memory

        @verbatim
        XMLPrinter printer;
        doc.Print( &printer );
        SomeFunction( printer.CStr() );
        @endverbatim

        Print to a File

        You provide the file pointer.
        @verbatim
        XMLPrinter printer( fp );
        doc.Print( &printer );
        @endverbatim

        Print without a XMLDocument

        When loading, an XML parser is very useful. However, sometimes
        when saving, it just gets in the way. The code is often set up
        for streaming, and constructing the DOM is just overhead.

        The Printer supports the streaming case. The following code
        prints out a trivially simple XML file without ever creating
        an XML document.

        @verbatim
        XMLPrinter printer( fp );
        printer.OpenElement( "foo" );
        printer.PushAttribute( "foo", "bar" );
        printer.CloseElement();
        @endverbatim
*/
class TINYXML2_LIB XMLPrinter : public XMLVisitor
{
public:
    /** Construct the printer. If the FILE* is specified,
        this will print to the FILE. Else it will print
        to memory, and the result is available in CStr().
        If 'compact' is set to true, then output is created
        with only required whitespace and newlines.
    */
    explicit XMLPrinter( const FILE* const file= nullptr, bool compact = false, int32_t depth = 0 );
    virtual ~XMLPrinter() = default;

    /** If streaming, write the BOM and declaration. */
    void PushHeader( bool writeBOM, bool writeDec );
    /** If streaming, start writing an element.
        The element must be closed with CloseElement()
    */
    void OpenElement( const char_t* name, bool compactMode = false );
    /// If streaming, add an attribute to an open element.
    void PushAttribute( const char_t* name, const char_t* value );
    void PushAttribute( const char_t* name, int32_t v );
    void PushAttribute( const char_t* name, uint32_t v );
    void PushAttribute( const char_t* name, int64_t v );
    void PushAttribute( const char_t* name, uint64_t v );
    void PushAttribute( const char_t* name, bool v );
    void PushAttribute( const char_t* name, float64_t v );
    /// If streaming, close the Element.
    virtual void CloseElement( bool compactMode = false );

    /// Add a text node.
    void PushText( const char_t* text, bool cdata = false );
    /// Add a text node from an integer.
    void PushText( int32_t value );
    /// Add a text node from an unsigned.
    void PushText( uint32_t value );
    /// Add a text node from a signed 64bit integer.
    void PushText( int64_t value );
    /// Add a text node from an unsigned 64bit integer.
    void PushText( uint64_t value );
    /// Add a text node from a bool.
    void PushText( bool value );
    /// Add a text node from a float.
    void PushText( float32_t value );
    /// Add a text node from a double.
    void PushText( float64_t value );

    /// Add a comment
    void PushComment( const char_t* comment );

    void PushDeclaration( const char_t* value );
    void PushUnknown( const char_t* value );

    virtual bool VisitEnter( const XMLDocument& /*doc*/ );
    virtual bool VisitExit( const XMLDocument& /*doc*/ )
    {
        return true;
    }

    virtual bool VisitEnter( const XMLElement& element, const XMLAttribute* attribute );
    virtual bool VisitExit( const XMLElement& element );

    virtual bool Visit( const XMLText& text );
    virtual bool Visit( const XMLComment& comment );
    virtual bool Visit( const XMLDeclaration& declaration );
    virtual bool Visit( const XMLUnknown& unknown );

    /**
        If in print to memory mode, return a pointer to
        the XML file in memory.
    */
    const char_t* CStr() const
    {
        return _buffer.Mem();
    }
    /**
        If in print to memory mode, return the size
        of the XML file in memory. (Note the size returned
        includes the terminating null.)
    */
    int32_t CStrSize() const
    {
        return _buffer.Size();
    }
    /**
        If in print to memory mode, reset the buffer to the
        beginning.
    */
    void ClearBuffer( bool resetToFirstElement = true )
    {
        _buffer.Clear();
        _buffer.Push(0);
        _firstElement = resetToFirstElement;
    }

protected:
    virtual bool CompactMode( const XMLElement& )	{ return _compactMode; }

    /** Prints out the space before an element. You may override to change
            the space and tabs used. A PrintSpace() override should call Print().
        */
    virtual void PrintSpace( int32_t depth );
    void Print( const char_t* format, ... );
    inline void Write(const char_t* data);
    void Write( const char_t* data, size_t size );
    void Putc( char_t ch );

    void SealElementIfJustOpened();
//    DynArray< const char_t*, 10 > _stack;

private:
    // Prohibit cloning, intentionally not implemented
    XMLPrinter( const XMLPrinter& ) = delete;
    XMLPrinter& operator=( const XMLPrinter& ) = delete;

    void PrintString( const char_t*, bool restricted );	// prints out, after detecting entities.

    FILE*   _fp;
    int32_t _depth;
    int32_t _textDepth;
    bool    _compactMode;
    bool    _firstElement;
    bool    _processEntities;
    bool    _elementJustOpened;

    enum
    {
        ENTITY_RANGE = 64,
        BUF_SIZE = 200
    };
    bool _entityFlag[ENTITY_RANGE] {};
    bool _restrictedEntityFlag[ENTITY_RANGE] {};

    DynArray< char_t, 20 > _buffer;
    DynArray< const char_t*, 10 > _stack;
};


}	// simproxml

#if defined(_MSC_VER)
#   pragma warning(pop)
#endif

#endif // SIMPROXML_INCLUDED
