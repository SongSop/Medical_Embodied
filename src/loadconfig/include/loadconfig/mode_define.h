#ifndef GLOABL_VARIABLE_H
#define GLOABL_VARIABLE_H

#include <string>

namespace INTERACTION
{
    enum{
        ALERT,
        PASSIVE,
        INTERUPT
    };
} // namespace INTERACTION

namespace NAVIGATION
{
    enum{
        GOAL,
        STOP,
        DOCK
    };
}

namespace DETECT
{
    enum{
        AREA,
        BED
    };
}

#endif
