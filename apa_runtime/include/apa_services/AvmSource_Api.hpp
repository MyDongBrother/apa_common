/**
 * @file AvmSource_Api.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-10-27
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-10-27 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#pragma once
#include <memory>
#include <vector>
#include <cstdint>

class AvmSource
{
  public:
    AvmSource();
    ~AvmSource();

    /**
     * @brief Read one frame from shared memory (blocking)
     * @param[out] data output image data (shared pointer)
     */
    void Read(std::shared_ptr<std::vector<uint8_t>> &data);

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl_; // PIMPL idiom
};
