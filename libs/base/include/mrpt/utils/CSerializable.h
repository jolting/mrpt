/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CSERIALIZABLE_H
#define  CSERIALIZABLE_H

#if MRPT_HAS_MATLAB
typedef struct mxArray_tag mxArray; //!< Forward declaration for mxArray (avoid #including as much as possible to speed up compiling)
#endif
#include <typeinfo>
#include <memory>

namespace mrpt
{
namespace utils
{
  class Any
  {
    class StorageBase
    {
    public:
      virtual ~StorageBase(){}
    };
  
    template<class T>
    class Storage : public StorageBase
    {
    public:
      Storage(const T& t) :t(t) {}
      Storage(T&& t) :t(t) {}
      ~Storage() override{}
      T t;
    };

  public:
    Any() : valid(false), m_typeInfo(&typeid(nullptr)){}
    ~Any(){}
    template<class V>
    Any(V&& v)
    {
      valid = true;
      m_ptr.reset(new Storage<V>(v));
      m_typeInfo = &typeid(V);
    }
    template<class V>
    Any(const V& v)
    {
      valid = true;
      m_ptr.reset(new Storage<V>(v));
      m_typeInfo = &typeid(V);
    }

    const std::type_info& type() const
    {
      return *m_typeInfo;
    }

    template<class T>
    T& get_unsafe()
    {
      return dynamic_cast<Storage<T>*>(m_ptr.get())->t;
    }
    template<class T>
    const T& get_unsafe() const
    {
      return dynamic_cast<Storage<T>*>(m_ptr.get())->t;
    }

  private:
    bool valid;
    std::unique_ptr<StorageBase> m_ptr;
    const std::type_info *m_typeInfo;
  };
  template<class V>V any_cast(const Any& operand)
  {
    //if(!valid || operand.type() != typeid(V))
    // throw
    return operand.get_unsafe<V>();
  }
}
} // End of namespace

#endif
