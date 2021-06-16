#ifndef LOOKUPARRAY_H
#define LOOKUPARRAY_H

#include <stdint.h>
#include <cstring>
#include <assert.h>

class LookupArray {
public:
  /**
   * Constructs lookup array
   */
  LookupArray() : m_pArray(NULL), m_Capacity(0), m_Size(0) {}

  /**
   * Destructor
   */
  virtual ~LookupArray() {
    assert(m_pArray != NULL);

    delete[] m_pArray;
    m_pArray = NULL;
  }

public:
  /**
   * Clear array
   */
  void Clear() { memset(m_pArray, 0, sizeof(int32_t) * m_Capacity); }

  /**
   * Gets size of array
   * @return array size
   */
  int32_t GetSize() const { return m_Size; }

  /**
   * Sets size of array (resize if not big enough)
   * @param size
   */
  void SetSize(int32_t size) {
    assert(size != 0);

    if (size > m_Capacity) {
      if (m_pArray != NULL) {
        delete[] m_pArray;
      }
      m_Capacity = size;
      m_pArray = new int32_t[m_Capacity];
    }

    m_Size = size;
  }

  /**
   * Gets reference to value at given index
   * @param index
   * @return reference to value at index
   */
  inline int32_t &operator[](int32_t index) {
    assert(index < m_Size);

    return m_pArray[index];
  }

  /**
   * Gets value at given index
   * @param index
   * @return value at index
   */
  inline int32_t operator[](int32_t index) const {
    assert(index < m_Size);

    return m_pArray[index];
  }

  /**
   * Gets array pointer
   * @return array pointer
   */
  inline int32_t *GetArrayPointer() { return m_pArray; }

  /**
   * Gets array pointer
   * @return array pointer
   */
  inline int32_t *GetArrayPointer() const { return m_pArray; }

private:
  int32_t *m_pArray;
  int32_t m_Capacity;
  int32_t m_Size;
}; // LookupArray

#endif
