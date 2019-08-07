#ifndef NAME_H
#define NAME_H

#include <string>

class Name {
public:
  /**
   * Constructor
   */
  Name() {}

  /**
   * Constructor
   */
  Name(const std::string &rName) { Parse(rName); }

  /**
   * Constructor
   */
  Name(const Name &rOther) : m_Name(rOther.m_Name), m_Scope(rOther.m_Scope) {}

  /**
   * Destructor
   */
  virtual ~Name() {}

public:
  /**
   * Gets the name of this name
   * @return name
   */
  inline const std::string &GetName() const { return m_Name; }

  /**
   * Sets the name
   * @param rName name
   */
  inline void SetName(const std::string &rName) {
    std::string::size_type pos = rName.find_last_of('/');
    if (pos != 0 && pos != std::string::npos) {
      throw "Name can't contain a scope!";
    }

    m_Name = rName;
  }

  /**
   * Gets the scope of this name
   * @return scope
   */
  inline const std::string &GetScope() const { return m_Scope; }

  /**
   * Sets the scope of this name
   * @param rScope scope
   */
  inline void SetScope(const std::string &rScope) { m_Scope = rScope; }

  /**
   * Returns a string representation of this name
   * @return string representation of this name
   */
  inline std::string ToString() const {
    if (m_Scope == "") {
      return m_Name;
    } else {
      std::string name;
      name.append("/");
      name.append(m_Scope);
      name.append("/");
      name.append(m_Name);

      return name;
    }
  }

public:
  /**
   * Assignment operator.
   */
  Name &operator=(const Name &rOther) {
    if (&rOther != this) {
      m_Name = rOther.m_Name;
      m_Scope = rOther.m_Scope;
    }

    return *this;
  }

  /**
   * Equality operator.
   */
  bool operator==(const Name &rOther) const {
    return (m_Name == rOther.m_Name) && (m_Scope == rOther.m_Scope);
  }

  /**
   * Inequality operator.
   */
  bool operator!=(const Name &rOther) const { return !(*this == rOther); }

  /**
   * Less than operator.
   */
  bool operator<(const Name &rOther) const {
    return ToString() < rOther.ToString();
  }

  /**
   * Write Name onto output stream
   * @param rStream output stream
   * @param rName to write
   */
  friend inline std::ostream &operator<<(std::ostream &rStream,
                                         const Name &rName) {
    rStream << rName.ToString();
    return rStream;
  }

private:
  /**
   * Parse the given string into a Name object
   * @param rName name
   */
  void Parse(const std::string &rName) {
    std::string::size_type pos = rName.find_last_of('/');

    if (pos == std::string::npos) {
      m_Name = rName;
    } else {
      m_Scope = rName.substr(0, pos);
      m_Name = rName.substr(pos + 1, rName.size());

      // remove '/' from m_Scope if first!!
      if (m_Scope.size() > 0 && m_Scope[0] == '/') {
        m_Scope = m_Scope.substr(1, m_Scope.size());
      }
    }
  }

  /**
   * Validates the given string as a Name
   * @param rName name
   */
  void Validate(const std::string &rName) {
    if (rName.empty()) {
      return;
    }

    char c = rName[0];
    if (IsValidFirst(c)) {
      for (size_t i = 1; i < rName.size(); ++i) {
        c = rName[i];
        if (!IsValid(c)) {
          throw "Invalid character in name. Valid characters must be "
                          "within the ranges A-Z, a-z, 0-9, '/', '_' and '-'.";
        }
      }
    } else {
      throw "Invalid first character in name. Valid characters must "
                      "be within the ranges A-Z, a-z, and '/'.";
    }
  }

  /**
   * Whether the character is valid as a first character (alphanumeric or /)
   * @param c character
   * @return true if the character is a valid first character
   */
  inline bool IsValidFirst(char c) { return (isalpha(c) || c == '/'); }

  /**
   * Whether the character is a valid character (alphanumeric, /, _, or -)
   * @param c character
   * @return true if the character is valid
   */
  inline bool IsValid(char c) {
    return (isalnum(c) || c == '/' || c == '_' || c == '-');
  }

private:
  std::string m_Name;
  std::string m_Scope;
};

#endif
