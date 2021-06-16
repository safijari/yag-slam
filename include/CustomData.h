#ifndef CUSTOM_DATA_H
#define CUSTOM_DATA_H
#include "Object.h"

class CustomData : public Object
{
  public:
  /**
   * Constructor
   */
  CustomData()
    : Object()
  {
  }

  /**
   * Destructor
   */
  virtual ~CustomData()
  {
  }

public:
  /**
   * Write out this custom data as a string
   * @return string representation of this data object
   */
  virtual const std::string Write() const = 0;

  /**
   * Read in this custom data from a string
   * @param rValue string representation of this data object
   */
  virtual void Read(const std::string& rValue) = 0;

private:
  CustomData(const CustomData&);
  const CustomData& operator=(const CustomData&);
};

/**
 * Type declaration of CustomData vector
 */
typedef std::vector<CustomData*> CustomDataVector;

#endif
