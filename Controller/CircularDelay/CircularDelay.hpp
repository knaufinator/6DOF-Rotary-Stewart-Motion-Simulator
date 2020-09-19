/**
 * @copyright  GPL V3
 *
 * Circular delay software library. Here data can be stored and retrieved is a
 * LiFo manner. Copyright (C) 2018 Jimmy van den Berg
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdexcept>
#include <cstdlib>
/**
 * @brief      A class that functions as a sample buffer.
 * @details    You can use this to insert samples and use the get function to
 *             get a sample from the past.
 *
 * @tparam     type  Type of sample that needs to be stored.
 * @tparam     size  Size of how big the history buffer is.
 */
template<typename type, size_t size>
class CircularDelay{
public:
	CircularDelay();
	class iterator{
	public:
		typedef iterator self_type;
		typedef std::bidirectional_iterator_tag iterator_category;
		typedef int difference_type;
		iterator(const CircularDelay<type, size>::iterator& it):data_(it.data_), ptr_(it.ptr_){}
		self_type operator++() {
			if(++ptr_ == data_ + size + 1){
				ptr_ = data_;
			}
			return *this;
		}
		self_type operator++(int) {self_type ret = *this; ++*this; return ret;}
		self_type operator--() {
			if(ptr_ == data_){
				ptr_ = data_ + size;
			}
			else{
				--ptr_;
			}
			return *this;
		}
		self_type operator--(int) {self_type ret = *this; --*this; return ret;}
		type& operator*() { return *ptr_; }
		type* operator->() { return ptr_; }
		type& operator[](unsigned int index){
			// Note that ptr_ - data_ is converting ptr_ to index value;
			index += ptr_ - data_;
			if(index >= size + 1){
				index -= size + 1;
			}
			return data_[index];
		}
		bool operator==(const self_type& rhs) { return ptr_ == rhs.ptr_; }
		bool operator!=(const self_type& rhs) { return ptr_ != rhs.ptr_; }
	private:
		iterator(type* data, type* ptr) : data_(data), ptr_(ptr) { }
		friend class CircularDelay;
		type* data_ = nullptr;
		type* ptr_ = nullptr;
	};
	class const_iterator{
	public:
		typedef const_iterator self_type;
		typedef std::bidirectional_iterator_tag iterator_category;
		typedef int difference_type;
		const_iterator(const CircularDelay<type, size>::const_iterator& it):data_(it.data_), ptr_(it.ptr_){}
		self_type operator++() {
			if(++ptr_ == data_ + size + 1){
				ptr_ = data_;
			}
			return *this;
		}
		self_type operator++(int) {self_type ret = *this; ++*this; return ret;}
		self_type operator--() {
			if(ptr_ == data_){
				ptr_ = data_ + size;
			}
			else{
				--ptr_;
			}
			return *this;
		}
		self_type operator--(int) {self_type ret = *this; --*this; return ret;}
		const type& operator*() { return *ptr_; }
		const type* operator->() { return ptr_; }
		const type& operator[](unsigned int index){
			// Note that ptr_ - data_ is converting ptr_ to index value;
			index += ptr_ - data_;
			if(index >= size + 1){
				index -= size + 1;
			}
			return data_[index];
		}
		bool operator==(const self_type& rhs) { return ptr_ == rhs.ptr_; }
		bool operator!=(const self_type& rhs) { return ptr_ != rhs.ptr_; }
	private:
		const_iterator(type* data, type* ptr) : data_(data), ptr_(ptr) { }
		friend class CircularDelay;
		type* data_ = nullptr;
		type* ptr_ = nullptr;
	};
	class reverse_iterator{
	public:
		typedef reverse_iterator self_type;
		typedef std::bidirectional_iterator_tag iterator_category;
		typedef int difference_type;
		reverse_iterator(const CircularDelay<type, size>::reverse_iterator& it):data_(it.data_), ptr_(it.ptr_){}
		self_type operator++() {
			if(ptr_ == data_){
				ptr_ = data_ + size;
			}
			else{
				--ptr_;
			}
			return *this;
		}
		self_type operator++(int) {self_type ret = *this; ++*this; return ret;}
		self_type operator--() {
			if(++ptr_ == data_ + size + 1){
				ptr_ = data_;
			}
			return *this;
		}
		self_type operator--(int) {self_type ret = *this; --*this; return ret;}
		type& operator*() { return *ptr_; }
		type* operator->() { return ptr_; }
		type& operator[](int index){
			// Convert ptr_ to index value;
			index = ptr_ - data_ - index;
			if(index < 0){
				index += size + 1;
			}
			return data_[index];
		}
		bool operator==(const self_type& rhs) { return ptr_ == rhs.ptr_; }
		bool operator!=(const self_type& rhs) { return ptr_ != rhs.ptr_; }
	private:
		reverse_iterator(type* data, type* ptr) : data_(data), ptr_(ptr) { }
		friend class CircularDelay;
		type* data_ = nullptr;
		type* ptr_ = nullptr;
	};
	class const_reverse_iterator{
	public:
		typedef const_reverse_iterator self_type;
		typedef std::bidirectional_iterator_tag iterator_category;
		typedef int difference_type;
		const_reverse_iterator(const CircularDelay<type, size>::const_reverse_iterator& it):data_(it.data_), ptr_(it.ptr_){}
		self_type operator++() {
			if(ptr_ == data_){
				ptr_ = data_ + size;
			}
			else{
				--ptr_;
			}
			return *this;
		}
		self_type operator++(int) {self_type ret = *this; ++*this; return ret;}
		self_type operator--() {
			if(++ptr_ == data_ + size + 1){
				ptr_ = data_;
			}
			return *this;
		}
		self_type operator--(int) {self_type ret = *this; --*this; return ret;}
		const type& operator*() { return *ptr_; }
		const type* operator->() { return ptr_; }
		const type& operator[](int index){
			// Convert ptr_ to index value;
			index = ptr_ - data_ - index;
			if(index < 0){
				index += size + 1;
			}
			return data_[index];
		}
		bool operator==(const self_type& rhs) { return ptr_ == rhs.ptr_; }
		bool operator!=(const self_type& rhs) { return ptr_ != rhs.ptr_; }
	private:
		const_reverse_iterator(type* data, type* ptr) : data_(data), ptr_(ptr) { }
		friend class CircularDelay;
		type* data_ = nullptr;
		type* ptr_ = nullptr;
	};
	type push(type input);
	type get(size_t delay);
	iterator end(){return setIterator;}
	iterator begin(){
		iterator it(setIterator);
		++it;
		return it;
	}
	reverse_iterator rend(){return reverse_iterator(data, setIterator.ptr_);}
	reverse_iterator rbegin(){
		reverse_iterator it(data, setIterator.ptr_);
		++it;
		return it;
	}
private:
	type data[size + 1];
	iterator setIterator = iterator(data, data);
};

/**
 * @brief      Constructor that initializes that buffer and its set index.
 *
 * @tparam     type  Type of sample that needs to be stored.
 * @tparam     size  Size of how big the history buffer is.
 */
template<typename type, size_t size>
CircularDelay<type, size>::CircularDelay(){
	for (size_t i = 0; i < size + 1; ++i){
		data[i] = 0;
	}
}

/**
 * @brief      With this function you can insert a new sample into the buffer.
 *
 * @param      input  Sample to push into.
 *
 * @tparam     type   Type of sample that needs to be stored.
 * @tparam     size   Size of how big the history buffer is.
 *
 * @return     Value that has been pushed.
 */
template<typename type, size_t size>
type CircularDelay<type, size>::push(type input){
	*setIterator = input;
	setIterator++;
	return input;
}

/**
 * @brief      With this function you can retrieve a sample from the past.
 * @details    Maximum delay is the size of the CircularDelay - 1.
 *
 * @param      delay  How many samples you ago you want to get.
 *
 * @tparam     type   Type of sample that needs to be stored.
 * @tparam     size   Size of how big the history buffer is.
 *
 * @return     The sample of delay ago.
 */
template<typename type, size_t size>
type CircularDelay<type, size>::get(size_t delay){
	if(delay >= size + 1)
		throw(std::domain_error("Tried to get a value that is longer ago than the size of a CircularDelay."));
	reverse_iterator itRBegin(rbegin());
	return itRBegin[delay];
}
