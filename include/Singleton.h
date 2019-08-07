#ifndef SINGLETON_H
#define SINGLETON_H

#include <cstdlib>
/**
 * Singleton class ensures only one instance of T is created
 */
template <class T> class Singleton {
public:
  /**
   * Constructor
   */
  Singleton() : m_pPointer(NULL) {}

  /**
   * Destructor
   */
  virtual ~Singleton() { delete m_pPointer; }

  /**
   * Gets the singleton
   * @return singleton
   */
  T *Get() {
    // #ifdef USE_POCO
    //     Poco::FastMutex::ScopedLock lock(m_Mutex);
    // #endif
    if (m_pPointer == NULL) {
      m_pPointer = new T;
    }

    return m_pPointer;
  }

private:
  T *m_pPointer;

  // #ifdef USE_POCO
  //   Poco::FastMutex m_Mutex;
  // #endif

private:
  Singleton(const Singleton &);
  const Singleton &operator=(const Singleton &);
};

#endif
