/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_COMMON_FACTORY_H_
#define DART_COMMON_FACTORY_H_

#include <map>
#include <string>
#include <functional>

#include "dart/common/Singleton.h"

namespace dart {
namespace common {

/// \brief Factory is implementation of Factory pattern
// TODO: Singleton vs static class
template <class Base>
class Factory
{
public:
  /// \brief Register object
  static void registerObject(const std::string& type,
                             std::function<Base*()> _registerFunction)
  {
    // If ret.second is true, then name is new key and ret.first is the iterator
    // pointing the new key. If ret.second is false, then reg already has name
    // and ret.first is the iterator point the first key of reg.
    std::pair<typename ObjectMap::iterator, bool> ret =
        getOrCreateObjectMap()->insert(
          typename ObjectMap::value_type(
            type,
            _registerFunction));

    if (ret.second == false)
    {
      // This means there already is a component registered to
      // this name. You should handle this error as you see fit.
      throw;
    }
  }

  /// \brief Unregister object
  static void unregisterObject(const std::string& type)
  {
    Factory::getOrCreateObjectMap()->erase(type);
  }

  /// \brief Create object
  static Base* createObject(const std::string& _type)
  {
    Factory::ObjectMap* map = Factory::getOrCreateObjectMap();
    typename ObjectMap::iterator it = map->find(_type);
    if (it != map->end())
      return (it->second());

    return nullptr;
  }

  /// \brief Return true if TEST_T and _type are match, return false otherwise
  template<typename TEST_T>
  static bool isValidType(const std::string& _type)
  {
    Base* dummy = createObject(_type);
    TEST_T* dummyCast = dynamic_cast<TEST_T*>(dummy);

    if (dummyCast != nullptr)
    {
      delete dummy;
      return true;
    }
    else
    {
      delete dummy;
      return false;
    }
  }

private:
  /// \brief Typedef of ObjectMap
  typedef std::map<std::string, std::function<Base*()>> ObjectMap;

private:
  /// \brief Constructor
  Factory() = delete;

  /// \brief Get or create object map
  static ObjectMap* getOrCreateObjectMap()
  {
    if (mObjectMap == nullptr)
      mObjectMap = new ObjectMap();

    return mObjectMap;
  }

private:
  /// \brief Object map
  static ObjectMap* mObjectMap;
};

template<class Base>
typename Factory<Base>::ObjectMap* Factory<Base>::mObjectMap = nullptr;

/// \brief
template<class Base, class Derived>
struct RegistryEntry
{
public:
  /// \brief Return singlton instance
  ///
  /// Because I use a singleton here, even though `COMPONENT_REGISTER`
  /// is expanded in multiple translation units, the constructor
  /// will only be executed once. Only this cheap `Instance` function
  /// (which most likely gets inlined) is executed multiple times.
  static RegistryEntry<Base, Derived>& getInstance(const std::string& name)
  {
    static RegistryEntry<Base, Derived> inst(name);
    return inst;
  }

private:
  /// \brief Constructor
  RegistryEntry(const std::string& _name)
  {
    // Register the class factory function
    Factory<Base>::registerObject(_name,
                            [](void) -> Base* { return new Derived(); });
  }

  /// \brief Constructor
  RegistryEntry(const RegistryEntry<Base, Derived>&) = delete;

  /// \brief Assignmen operator
  RegistryEntry& operator=(const RegistryEntry<Base, Derived>&) = delete;
};

}  // namespace common
}  // namespace dart

#define DART_FACTORY_CREATABLE(BASE, TYPE)                                     \
  public:                                                                      \
    friend class dart::common::RegistryEntry<BASE, TYPE>;

#define DART_FACTORY_REGISTER(BASE, TYPE, NAME)                                \
namespace                                                                      \
{                                                                              \
  template<class BASE, class TYPE>                                             \
  class Object##BASE##TYPE##Registration                                       \
  {                                                                            \
    static const dart::common::RegistryEntry<BASE, TYPE>& reg;                 \
  };                                                                           \
                                                                               \
  template<>                                                                   \
  const dart::common::RegistryEntry<BASE, TYPE>&                               \
      Object##BASE##TYPE##Registration<BASE, TYPE>::reg                        \
          = dart::common::RegistryEntry<BASE, TYPE>::getInstance(NAME);        \
}

#endif  // DART_COMMON_FACTORY_H_
