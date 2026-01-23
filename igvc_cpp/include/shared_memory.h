#ifndef IGVC_CPP_SHARED_MEMORY_H
#define IGVC_CPP_SHARED_MEMORY_H

#pragma once
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdexcept>
#include <string>

class SharedMemory
{
public:
    SharedMemory(const std::string& name, size_t size)
        : m_name("/" + name),
          m_semName("/" + name + "_sem"),
          m_size(size)
    {
        m_fd = shm_open(m_name.c_str(), O_CREAT | O_RDWR, 0666);
        if (m_fd < 0) {
            throw std::runtime_error("shm_open failed");
        }

        if (ftruncate(m_fd, size) != 0) {
            throw std::runtime_error("ftruncate failed");
        }

        m_ptr = mmap(nullptr, size,
                     PROT_READ | PROT_WRITE,
                     MAP_SHARED, m_fd, 0);

        if (m_ptr == MAP_FAILED) {
            throw std::runtime_error("mmap failed");
        }

        m_sem = sem_open(m_semName.c_str(), O_CREAT, 0666, 1);
        if (m_sem == SEM_FAILED) {
            throw std::runtime_error("sem_open failed");
        }
    }

    ~SharedMemory()
    {
        munmap(m_ptr, m_size);
        close(m_fd);
        sem_close(m_sem);
    }

    [[nodiscard]] void* Data() const { return m_ptr; }

    void Lock() const { sem_wait(m_sem); }
    void Unlock() const { sem_post(m_sem); }

private:
    std::string m_name;
    std::string m_semName;
    size_t m_size;
    int m_fd;
    void* m_ptr;
    sem_t* m_sem;
};


#endif