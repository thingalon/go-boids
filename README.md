Concurrent Boids implementation in golang
=========================================

This is an implementation of [Boids](http://en.wikipedia.org/wiki/Boids) written in golang. Each boid update runs in a goroutine, allowing for concurrent updates.

Installation
------------

This requires the following go-gettable libs: 
    go get github.com/go-gl/gl
    go get github.com/go-gl/glfw3
    go get github.com/go-gl/gltext

Before you go-get them, you'll need to ensure your system has mercurial, glew and glfw installed. If you're on OSX and use homebrew, you can install them like so: 
    brew install mercurial
    brew install glew
    brew install glfw3

Warning: this was written while learning golang, so this code is likely to contain ugly.

(Contains [Amble](http://www.fontsquirrel.com/fonts/amble), a free font by [Punchcut](http://www.punchcut.com/))
