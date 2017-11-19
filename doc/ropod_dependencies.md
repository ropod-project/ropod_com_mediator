Zyre
====

Follow instructions for **master** branches: https://github.com/zeromq/zyre

At the moment of writing (Oct, 2017), those correspond to:
* libsodium 1.0.15
* libzmq 4.2.2
* czmq 4.0.2
* zyre 2.0.0

```
    git clone --depth 1 -b stable https://github.com/jedisct1/libsodium.git
    cd libsodium
    ./autogen.sh && ./configure && make check
    sudo make install
    cd ..

    git clone git://github.com/zeromq/libzmq.git
    cd libzmq
    ./autogen.sh
    # do not specify "--with-libsodium" if you prefer to use internal tweetnacl
    # security implementation (recommended for development)
    ./configure --with-libsodium
    make check
    sudo make install
    sudo ldconfig
    cd ..

    git clone git://github.com/zeromq/czmq.git
    cd czmq
    ./autogen.sh && ./configure && make check
    sudo make install
    sudo ldconfig
    cd ..

    git clone git://github.com/zeromq/zyre.git
    cd zyre
    ./autogen.sh && ./configure && make check
    sudo make install
    sudo ldconfig
    cd ..
```

JSONCPP
=======

```
  apt-get install libjsoncpp-dev
```
