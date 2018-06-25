#include "abstractccinterface.hpp"

AbstractCCInterface::AbstractCCInterface(QObject *parent)
    : QObject(parent)
{

}

AbstractCCInterface::AbstractCCInterface(const AbstractCCInterface &intf)
    : QObject(intf.parent())
{

}

AbstractCCInterface::~AbstractCCInterface()
{
}
