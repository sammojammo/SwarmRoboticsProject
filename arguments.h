#ifndef ARGUMENTS_H_
#define ARGUMENTS_H_

/******************************************************************************/
/******************************************************************************/

#include "common.h"

/******************************************************************************/
/******************************************************************************/


class CArguments
{
public:
    CArguments(const char* pch_commandline);
    virtual ~CArguments();

    virtual const char* GetArgument(const char* pch_indicator);

    virtual bool        GetArgumentIsDefined(const char* pch_indicator);
    virtual int         GetArgumentAsInt(const char* pch_indicator);
    virtual const char* GetArgumentAsString(const char* pch_indicator);
    virtual double      GetArgumentAsDouble(const char* pch_indicator);

    virtual int         GetArgumentAsIntOr(const char* pch_indicator, int value);
    virtual const char* GetArgumentAsStringOr(const char* pch_indicator, const char* value);
    virtual double      GetArgumentAsDoubleOr(const char* pch_indicator, double value);

    virtual const char* GetCompleteString() const;

protected:
    char*    m_pchArgumentList;
    char     m_pchArgumentValue[512];
};

/******************************************************************************/
/******************************************************************************/

#endif
