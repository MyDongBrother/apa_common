//
// Copyright 2020 bm-intelligent.
//
#ifndef ZMQ_SUB_H_
#define ZMQ_SUB_H_

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

class Sub
{
  public:
    explicit Sub(std::string filepath)
    {
        if (filepath == "")
        {
            std::cout << "no zmq ipc file input" << std::endl;
        }
        file_path = filepath;
        if ((context = zmq_ctx_new()) == NULL)
        {
            std::cout << "sub context init failed" << std::endl;
        }
    }
    ~Sub()
    {
        zmq_close(suber);
        zmq_ctx_destroy(context);
    }
    void SetSub(bool inite) { sub_inite = inite; }
    int InitSub()
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
        if ((suber = zmq_socket(context, ZMQ_SUB)) == NULL)
        {
            std::cout << " sub socket init failed" << std::endl;
            zmq_ctx_destroy(context);
            return -1;
        }
        int hwm = 5;
        int rc  = zmq_setsockopt(suber, ZMQ_RCVHWM, &hwm, sizeof(hwm));
        if (rc < 0)
        {
            std::cout << "set recvhwm failed" << std::endl;
            return -1;
        }
        int rcvbuf = 1024 * 1024;
        rc         = zmq_setsockopt(suber, ZMQ_RCVBUF, &rcvbuf, sizeof(rcvbuf));
        if (rc < 0)
        {
            std::cout << "set recv buf failed" << std::endl;
            return -1;
        }
        zmq_setsockopt(suber, ZMQ_SUBSCRIBE, "", 0);
        if (zmq_connect(suber, addr.c_str()) < 0)
        {
            std::cout << "sub connect failed: " << zmq_strerror(errno) << std::endl;
            return -1;
        }
        std::cout << "connect to: " << addr << std::endl;
        sub_inite = true;
        return 0;
    }

    int IpcSub(uint8_t *data)
    {
        if (!sub_inite)
        {
            if (InitSub() == -1)
            {
                std::cout << "init sub failed!" << std::endl;
                return -1;
            }

            // int iRcvTimeout = 5000;
            // if (zmq_setsockopt(suber, ZMQ_RCVTIMEO, &iRcvTimeout, sizeof(iRcvTimeout))
            // < 0) {
            //    std::cout << "set time out failed" << std::endl;
            //    return 0;
            // }
        }
        zmq_msg_t msg;
        zmq_msg_init(&msg);
        int ret;
        // ret = zmq_msg_recv(&msg, suber, 0);
        ret = zmq_msg_recv(&msg, suber, ZMQ_DONTWAIT);
        if (ret < 0)
        {
            if (errno == EAGAIN)
            {
                // std::cout << "No message" << std::endl;
                ret = 0;
            }
            else
            {
                std::cout << "error = " << zmq_strerror(errno) << std::endl;
            }
        }
        if (ret > 0)
        {
            memcpy(data, zmq_msg_data(&msg), ret);
        }
        zmq_msg_close(&msg);
        return ret;
    }
    std::string file_path;
    bool sub_inite = false;
    void *context;
    void *suber;
    void *puber;
};

#endif // ZMQ_SUB_H_
