/*
This file is part of Yamabros.

Yamabros is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Yamabros is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Yamabros. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef YAMABROS_ROBOT_H
#define YAMABROS_ROBOT_H

#include <boost/shared_ptr.hpp>

#include <map>
#include <stdexcept>
#include <string>

namespace yamabros
{

class Robot
{
  /**
   * \brief Abstract base class for structures that encapsulate stored values.
   */
  struct Entry
  {
    /** \brief Reference-counted pointer type alias. */
    typedef boost::shared_ptr<Entry> P;

    /**
    \brief Virtual destructor. Enforces polymorphism. Do not remove.
    */
    virtual ~Entry()
    {
      // Do nothing.
    }
  };

  /**
   * \brief Template structure for encapsulating robot components.
   */
  template<class T>
  class Component: public Entry
  {
    /** \brief Reference-counted pointer to stored value. */
    boost::shared_ptr<T> value_;

  public:
    /**
    \brief Encapsulates the given pointer.

    The pointer is stored as a reference-counted smart pointer. When the reference
    count goes down to zero, the pointed object is deleted.
    */
    Component(T *pointer):
      value_(pointer)
    {
      // Nothing to do.
    }

    T &get()
    {
      return *value_;
    }
  };

  /** \brief Map of string-indexed components. */
  std::map<std::string, Entry::P> entries;

public:
  /**
   * \brief Initialize a ROS node on the current process.
   */
  void init(const std::string &name, int argc, char *argv[]);

  /**
   * \brief Retrieves a named component.
   */
  template<class T>
  T &get(const std::string &name);

  /**
   * \brief Sets a robot component to the given object.
   */
  template<class T>
  void set(const std::string &name, T *component);

  /**
   * \brief Puts the thread on spin.
   */
  void spin();
};

template<class T>
T &Robot::get(const std::string &name)
{
  Component<T> *component = dynamic_cast<Component<T>*>(entries[name].get());
  if (component == NULL)
    throw std::runtime_error("Component \"" + name + "\" not found");

  return component->get();
}

template<class T>
void Robot::set(const std::string &name, T *component)
{
  entries[name].reset(new Component<T>(component));
}

} // namespace yamabros

#endif
