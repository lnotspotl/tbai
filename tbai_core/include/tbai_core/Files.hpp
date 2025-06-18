#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace tbai {

/**
 * @brief Filesystem process-level lock with thread safety. This is both a cross-process and cross-thread
 * synchronization primitive. Uses both file-based locking for process synchronization and mutex-based locking for
 * thread synchronization.
 */
class FSLock {
   public:
    explicit FSLock(const std::string &fname);
    virtual ~FSLock();

    /**
     * @brief Lock the file produced by getLockPath()
     */
    void lock();

    /**
     * @brief Unlock the file produced by getLockPath()
     */
    void unlock();

    /**
     * @brief Check if the lock is held
     * @return True if the lock is held, false otherwise
     */
    bool holding() const;

    /**
     * @brief Get the path to the lock file
     * @param path Path to the file
     * @return Path to the lock file
     */
    virtual std::string getLockPath(const std::string &path) = 0;

    FSLock(const FSLock &) = delete;
    FSLock &operator=(const FSLock &) = delete;

   protected:
    int fd_ = -1;

    std::string filename_;
    std::string lockPath_;
    bool isLocked_;
    std::mutex &threadLock_;

    // Thread synchronization
    static std::mutex &getThreadLock(const std::string &key) {
        static std::mutex map_mutex;
        std::lock_guard<std::mutex> map_lock(map_mutex);  // prevents race condition when accessing thread_locks

        static std::unordered_map<std::string, std::mutex> thread_locks;
        return thread_locks[key];
    }
};

/**
 * @brief Filesystem cross-process and cross-thread lock for files.
 */
class FileLock : public FSLock {
   public:
    explicit FileLock(const std::string &path, bool createIfNotExists = false);
    std::string getLockPath(const std::string &path) override;
    using FSLock::lock;

    /**
     * @brief Lock the file given by path
     * @param path Path to the file
     * @param createIfNotExists If true, do not throw an error if the file does not exist, create it instead
     * @return FSLock instance
     */
    static std::unique_ptr<FileLock> lock(const std::string &path, bool createIfNotExists = false);
};

/**
 * @brief Filesystem process-level lock for folders. This is both a cross-process synchronization primitive and
 * thread-safe. Uses both file-based locking for process synchronization and mutex-based locking for thread
 * synchronization.
 */
class FolderLock : public FSLock {
   public:
    explicit FolderLock(const std::string &path);
    std::string getLockPath(const std::string &path) override;
    using FSLock::lock;

    /**
     * @brief Lock the folder given by path
     * @param path Path to the folder
     * @return FSLock instance
     */
    static std::unique_ptr<FolderLock> lock(const std::string &path);
};

}  // namespace tbai