#include <vrmapping/IdPool.h>
IdPool::IdPool()
{
    for (int i = 0; i <= 100000; i++)
    {
        id_pool.insert(i);
    }
}

IdPool::IdPool(int IdMax)
{
    if (IdMax < 1)
    {
        std::cout << "//////////ERROR//////////" << std::endl
                  << "[IdPool_Info]: The id Max is too small." << std::endl;
        return;
    }
    for (int i = 0; i <= IdMax; i++)
    {
        id_pool.insert(i);
    }
}

int IdPool::extractId()
{
    if (id_pool.empty())
    {
        std::cout << "//////////ERROR//////////" << std::endl
                  << "[IdPool_Info]: The id pool is empty." << std::endl;
        // throw std::out_of_range("The id pool is empty.");
    }

    int id = *id_pool.begin();
    id_pool.erase(id_pool.begin());
    return id;
}

void IdPool::addId(int id)
{
    id_pool.insert(id);
}

void IdPool::removeId(int id)
{
    id_pool.erase(id);
}