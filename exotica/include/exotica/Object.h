/*
 *      Author: Michael Camilleri
 * 
 * Copyright (c) 2016, University of Edinburgh
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of  nor the names of its contributors may be used to 
 *    endorse or promote products derived from this software without specific 
 *    prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 *
 */

#ifndef EXOTICA_OBJECT_H
#define EXOTICA_OBJECT_H

#include <exotica/Server.h>
#include <exotica/Tools.h>
#include <string>  //!< C++ type strings

#include "exotica/ObjectInitializer.h"
#include "exotica/Property.h"

namespace exotica
{
template <typename BO>
class Factory;

class Object
{
    template <typename BO>
    friend class Factory;

public:
    /**
       * \brief Constructor: default
       */
    Object()
        : ns_(""), debug_(false)
    {
        //!< Empty constructor
    }

    virtual ~Object()
    {
    }

    /**
       * \brief Type Information wrapper: must be virtual so that it is polymorphic...
       * @return String containing the type of the object
       */
    inline virtual std::string type()
    {
        return getTypeName(typeid(*this));
    }

    std::string getObjectName()
    {
        return object_name_;
    }

    void InstatiateObject(const Initializer& init)
    {
        ObjectInitializer oinit(init);
        object_name_ = oinit.Name;
        debug_ = oinit.Debug;
    }

    virtual std::string print(std::string prepend)
    {
        return prepend + "  " + object_name_ + " (" + type() + ")";
    }

    //	Namespace, i.e. problem/scene/...etc
    std::string ns_;
    std::string object_name_;
    bool debug_;
};
}
#endif
