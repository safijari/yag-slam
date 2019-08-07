#ifndef OBJECT_H
#define OBJECT_H

#include "Name.h"
#include <vector>

class NonCopyable {
private:
  NonCopyable(const NonCopyable &);
  const NonCopyable &operator=(const NonCopyable &);

protected:
  NonCopyable() {}

  virtual ~NonCopyable() {}
}; // class NonCopyable

/**
 * Abstract base class for Karto objects.
 */
class Object : public NonCopyable {
public:
  /**
   * Default constructor
   */
  Object();

  /**
   * Constructs an object with the given name
   * @param rName
   */
  Object(const Name &rName);

  /**
   * Default constructor
   */
  virtual ~Object();

public:
  /**
   * Gets the name of this object
   * @return name
   */
  inline const Name &GetName() const { return m_Name; }

  /**
   * Gets the class name of this object
   * @return class name
   */
  // virtual const char *GetClassName() const = 0;

  /**
   * Gets the type of this object
   * @return object type
   */
  // virtual kt_objecttype GetObjectType() const = 0;

  // /**
  //  * Gets the parameter manager of this dataset
  //  * @return parameter manager
  //  */
  // virtual inline ParameterManager *GetParameterManager() {
  //   return m_pParameterManager;
  // }

  /**
   * Gets the named parameter
   * @param rName name of parameter
   * @return parameter
   */
  // inline AbstractParameter *GetParameter(const std::string &rName) const {
  //   return m_pParameterManager->Get(rName);
  // }

  /**
   * Sets the parameter with the given name with the given value
   * @param rName name
   * @param value value
   */
  template <typename T>
  inline void SetParameter(const std::string &rName, T value);

  /**
   * Gets all parameters
   * @return parameters
   */
  // inline const ParameterVector &GetParameters() const {
  //   return m_pParameterManager->GetParameterVector();
  // }

private:
  Object(const Object &);
  const Object &operator=(const Object &);

private:
  Name m_Name;
  // ParameterManager *m_pParameterManager;
};

/**
 * Type declaration of Object vector
 */
typedef std::vector<Object *> ObjectVector;

#endif
