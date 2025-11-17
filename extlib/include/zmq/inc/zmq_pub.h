//
// Copyright 2020 bm-intelligent.
//
#ifndef ZMQ_PUB_H_
#define ZMQ_PUB_H_
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <zmq.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <vector>

class Pub
{
  public:
    Pub(std::string filepath)
    {
        if (filepath == "")
        {
            std::cout << "no zmq ipc file input" << std::endl;
        }
        file_path = filepath;
        if ((context = zmq_ctx_new()) == NULL)
        {
            std::cout << "pub context init failed" << std::endl;
        }
    }
    ~Pub()
    {
        zmq_close(puber);
        zmq_ctx_destroy(context);
    }
    int InitPub()
    {
        if (access(file_path.c_str(), F_OK) == -1)
        {
            if (creat(file_path.c_str(), 0755) < 0)
            {
                std::cout << "create file failed" << std::endl;
                return -1;
            }
        }
        std::string addr = "ipc://" + file_path;
        if ((puber = zmq_socket(context, ZMQ_PUB)) == NULL)
        {
            std::cout << " sub socket init failed" << std::endl;
            return -1;
        }
        int hwm = 10;
        int rc  = zmq_setsockopt(puber, ZMQ_SNDHWM, &hwm, sizeof(hwm));
        if (rc < 0)
        {
            std::cout << "set sndhwm failed" << std::endl;
            return -1;
        }
        int linger = 1000;
        rc         = zmq_setsockopt(puber, ZMQ_LINGER, &linger, sizeof(linger));
        if (rc < 0)
        {
            std::cout << "set linger failed" << std::endl;
            return -1;
        }
        int sndbuf = 16 * 1024;
        rc         = zmq_setsockopt(puber, ZMQ_SNDBUF, &sndbuf, sizeof(sndbuf));
        if (rc < 0)
        {
            std::cout << "set sndbuf failed" << std::endl;
            return -1;
        }
        if (zmq_bind(puber, addr.c_str()) < 0)
        {
            std::cout << "pub bind failed: " << errno << std::endl;
            return -1;
        }
        usleep(150000);
        pub_inite = true;
        return 0;
    }
    int IpcPub(uint8_t *data, int length)
    {
        if (!pub_inite)
        {
            if (InitPub() == -1)
            {
                std::cout << "pub init failed!" << std::endl;
                return 0;
            }
        }

        zmq_msg_t msg;
        int rc = 0;
        rc     = zmq_msg_init_size(&msg, length);
        if (rc != 0)
        {
            return -1;
        }

        memcpy(zmq_msg_data(&msg), data, length);
        if (zmq_msg_send(&msg, puber, 0) < 0)
        {
            std::cout << "send message faild: " << stderr << std::endl;
        }
        zmq_msg_close(&msg);
        return 0;
    }
    std::string file_path;
    bool pub_inite = false;
    void *context;
    void *puber;
};

#endif // ZMQ_PUB_H_
