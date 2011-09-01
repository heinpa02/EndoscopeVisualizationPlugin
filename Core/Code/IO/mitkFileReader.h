/*=========================================================================

Program:   Medical Imaging & Interaction Toolkit
Language:  C++
Date:      $Date$
Version:   $Revision$

Copyright (c) German Cancer Research Center, Division of Medical and
Biological Informatics. All rights reserved.
See MITKCopyright.txt or http://www.mitk.org/copyright.html for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notices for more information.

=========================================================================*/


#ifndef FILEREADER_H_HEADER_INCLUDED_C1E7E521
#define FILEREADER_H_HEADER_INCLUDED_C1E7E521

#include <MitkExports.h>

namespace mitk {

//##Documentation
//## @brief Interface class of readers that read from files
//## @ingroup Process
class MITK_CORE_EXPORT FileReader
{
  public:
    //##Documentation
    //## @brief Get the specified the file to load.
    //## 
    //## Either the FileName or FilePrefix plus FilePattern are used to read.
    virtual const char* GetFileName() const = 0;

    //##Documentation
    //## @brief Specify the file to load.
    //## 
    //## Either the FileName or FilePrefix plus FilePattern are used to read.
    virtual void SetFileName(const char* aFileName) = 0;

    //##Documentation
    //## @brief Get the specified file prefix for the file(s) to load. 
    //## 
    //## You should specify either a FileName or FilePrefix. Use FilePrefix if
    //## the data is stored in multiple files.
    virtual const char* GetFilePrefix() const = 0;

    //##Documentation
    //## @brief Specify file prefix for the file(s) to load.
    //## 
    //## You should specify either a FileName or FilePrefix. Use FilePrefix if
    //## the data is stored in multiple files.
    virtual void SetFilePrefix(const char* aFilePrefix) = 0;

    //##Documentation
    //## @brief Get the specified file pattern for the file(s) to load. The
    //## sprintf format used to build filename from FilePrefix and number.
    //## 
    //## You should specify either a FileName or FilePrefix. Use FilePrefix if
    //## the data is stored in multiple files.
    virtual const char* GetFilePattern() const = 0;

    /**
    @brief Specified file pattern for the file(s) to load. The sprintf
    format used to build filename from FilePrefix and number.
    
    You should specify either a FileName or FilePrefix. Use FilePrefix if
    the data is stored in multiple files. */
    virtual void SetFilePattern(const char* aFilePattern) = 0;

    /**
    @brief Specifies, whether the file reader also can 
    read a file from a memory buffer */
    virtual bool CanReadFromMemory(  );
    
    /**
    @brief Set/Get functions to advise the file reader to
    use a memory array for reading a file*/
    virtual void SetReadFromMemory( bool read );
    virtual bool GetReadFromMemory(  );

    /**
    @brief To be used along with a call of SetReadFromMemory(true). This sets 
    the memory buffer and the size from which the reader will read.*/
    virtual void SetMemoryBuffer(const char* dataArray, unsigned int size);


protected:
    FileReader();
    virtual ~FileReader();

    bool m_CanReadFromMemory;
    bool m_ReadFromMemory;

    const    char* m_MemoryBuffer;
    unsigned int   m_MemorySize;
public:

protected:
};
} // namespace mitk
#endif /* FILEREADER_H_HEADER_INCLUDED_C1E7E521 */


