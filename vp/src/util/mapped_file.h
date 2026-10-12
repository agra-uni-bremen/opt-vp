#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <string>

// Maps a whole file read only into memory. The mapping lives as long as the object.
class MappedFile {
	const char *data_ = nullptr;
	std::size_t size_ = 0;

	[[noreturn]] static void fail(const std::string &filename, const char *step) {
		throw std::runtime_error("cannot " + std::string(step) + " '" + filename + "': " + std::strerror(errno));
	}

public:
	explicit MappedFile(const std::string &filename) {
		int fd = open(filename.c_str(), O_RDONLY);
		if (fd < 0)
			fail(filename, "open");

		struct stat st;
		if (fstat(fd, &st) != 0) {
			int error = errno;
			close(fd);
			errno = error;
			fail(filename, "stat");
		}
		size_ = st.st_size;

		// mmap rejects a length of 0 with EINVAL, so an empty file fails here
		void *p = mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, fd, 0);
		if (p == MAP_FAILED) {
			int error = errno;
			close(fd);
			errno = error;
			fail(filename, "map");
		}
		data_ = static_cast<const char *>(p);
		// the mapping stays valid after the file descriptor is closed
		close(fd);
	}

	~MappedFile() {
		munmap(const_cast<char *>(data_), size_);
	}

	MappedFile(const MappedFile &) = delete;
	MappedFile &operator=(const MappedFile &) = delete;

	const char *data() const {
		return data_;
	}

	std::size_t size() const {
		return size_;
	}

	// A file that cannot be opened throws in the constructor, so an existing object is always open.
	bool is_open() const {
		return true;
	}
};
