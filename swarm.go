package main

import (
	"runtime"
	"math/rand"
	"math"
	"time"
	"os"
	
	glfw "github.com/go-gl/glfw3"
	"github.com/go-gl/gl"
	"github.com/go-gl/gltext"
)

const (
	World_Size = 200
	Half_World_Size = World_Size / 2

	Acceleration_Limit = 10
	Speed_Limit = 20
	
	Separation_Distance = 5
	Separation_Factor = 30
	Alignment_Distance = 6
	Alignment_Factor = 20
	Cohesion_Distance = 12
	Cohesion_Factor = 10
)

type Vector struct { 
	x, y float64 
}

func ( vec *Vector ) add( other Vector ) { 
	vec.x += other.x
	vec.y += other.y
}

func ( vec Vector ) plus( other Vector ) Vector {
	return Vector{ vec.x + other.x, vec.y + other.y };
}

func ( vec Vector ) minus( other Vector ) Vector {
	return Vector{ vec.x - other.x, vec.y - other.y };
}

func ( vec Vector ) divided_by( n float64 ) Vector {
	return Vector{ vec.x / n, vec.y / n };
}

func ( vec Vector ) distance_to( other Vector ) float64 {
	return math.Sqrt( ( other.x - vec.x ) * ( other.x - vec.x ) + ( other.y - vec.y ) * ( other.y - vec.y ) );
}

func ( vec Vector ) square_distance_to( other Vector ) float64 {
	return ( other.x - vec.x ) * ( other.x - vec.x ) + ( other.y - vec.y ) * ( other.y - vec.y );
}

func ( vec Vector ) square_magnitude() float64 {
	return vec.x * vec.x + vec.y * vec.y
}

func ( vec Vector ) times( n float64 ) Vector {
	return Vector{ vec.x * n, vec.y * n }
}

func ( vec Vector ) heading() float64 {
	return math.Atan2( vec.y, vec.x )
}

type Index_Vector struct {
	x, y int
}

func ( vec Vector ) to_index_vector() Index_Vector {
	return Index_Vector{ int( vec.x / 10 ), int( vec.y / 10 ) }
}

type Area struct { top, bottom, left, right float64 }
func ( area Area ) width() float64 { return area.right - area.left; }
func ( area Area ) height() float64 { return area.bottom - area.top; }

type Boid struct {
	position Vector
	velocity Vector
	last_updated time.Time
	color_index int
	color_factor float32
}

type State struct {
	boids []*Boid
	index Boid_Position_Index
}

type Color struct {
	r, g, b float32
}

type Boid_Position_Index map[Index_Vector][]*Boid

func ( bpi Boid_Position_Index ) add_boid( boid* Boid ) {
	key := boid.position.to_index_vector()
	if _, ok := bpi[ key ]; ! ok {
		bpi[ key ] = make( []*Boid, 0, 10 )
	}
	bpi[ key ] = append( bpi[ key ], boid )
}

func ( bpi Boid_Position_Index ) boids_near( boid* Boid, distance float64, callback func( boid* Boid, distance float64 ) ) {
	distance_vector := Vector{ distance, distance }
	from := boid.position.minus( distance_vector ).to_index_vector()
	to := boid.position.plus( distance_vector ).to_index_vector()
	
	for x := from.x; x <= to.x; x++ {
		for y:= from.y; y <= to.y; y++ {
			if cell_boids, ok := bpi[ Index_Vector{ x, y } ]; ok {
				for _, cell_boid := range cell_boids {
					if ( boid != cell_boid ) {
						boid_distance := boid.position.distance_to( cell_boid.position )
						if ( boid_distance <= distance ) {
							callback( cell_boid, boid_distance )
						}
					}
				}
			}
		}
	}
}

var (
	running = true

	font *gltext.Font
	window_width = 1024
	window_height = 768
	visible_area Area

	boid_colors = [6]Color{
		{ float32(233)/255, float32(109)/255, float32(99) /255  },
		{ float32(127)/255, float32(202)/255, float32(159)/255 },
		{ float32(244)/255, float32(186)/255, float32(112)/255 },
		{ float32(133)/255, float32(193)/255, float32(245)/255 },
		{ float32(74) /255, float32(120)/255, float32(156)/255 },
		{ float32(252)/255, float32(254)/255, float32(253)/255 },
	}

	last_frame_time time.Time
	average_frame_time float64

	population_size = 1000
	attractor_position Vector
	attractor_polarity = float64( 1 )
	attraction_distance = float64( 60 )
	attraction_factor = float64( 80000 )
)

func init() {
	//	Ensure main.main runs in the main OS thread, to prevent OpenGL having a heart attack.
	runtime.LockOSThread()
}

func main() {
	rand.Seed( time.Now().UTC().UnixNano() )
	last_frame_time = time.Now()

	//	Initialize GLFW lib.
	if ! glfw.Init() {
		panic( "Failed to initialize GLFW3" )
	}
	defer glfw.Terminate()

	//	Create a window.
	window, err := glfw.CreateWindow( window_width, window_height, "Swarm", nil, nil )
	if err != nil {
		panic( err )
	}
	defer window.Destroy()

	window.SetFramebufferSizeCallback( on_resize )
	window.SetKeyCallback( on_key )
	window.SetCursorPositionCallback( on_mouse_move )	
	window.SetCloseCallback( on_close )

	window.MakeContextCurrent()
	glfw.SwapInterval( 1 )
	
	on_resize( window, window_width, window_height )

	configure_gl()
	
	//	Load a font
	fd, err := os.Open( "amble.ttf" )
	if err != nil {
		panic( err )
	}
	font, err = gltext.LoadTruetype( fd, 35, 32, 127, gltext.LeftToRight )
	if err != nil {
		panic( err )
	}
	fd.Close()

	state_channel := make( chan *State )
	go loop_state( state_channel )

	for running {
		draw_state( <- state_channel )
		window.SwapBuffers()
		glfw.PollEvents();
	}
}

func loop_state( state_channel chan *State ) {
	var last_state *State = nil;
	boid_updates := make( chan *Boid, 1000 )

	for running {
		new_state := new( State )
		new_state.boids = make( []*Boid, population_size )
		new_state.index = make( Boid_Position_Index )

		for i := 0; i < len( new_state.boids ); i++ {
			if last_state != nil && len( last_state.boids ) > i {
				go update_boid( last_state, last_state.boids[ i ], boid_updates )
			} else {
				go create_boid( boid_updates )
			}
		}

		for i := 0; i < len( new_state.boids ); i++ {
			new_state.boids[ i ] = <- boid_updates
			new_state.index.add_boid( new_state.boids[ i ] )
		}
		
		state_channel <- new_state
		last_state = new_state
	}
}

func update_boid( from_state *State, from_boid *Boid, out_channel chan *Boid ) {
	updated_boid := new( Boid );
	*updated_boid = *from_boid
	
	//	Calculate elapsed time since last update (local to each boid)
	now := time.Now()
	elapsed_nanoseconds := now.Sub( from_boid.last_updated )
	elapsed_seconds := float64( elapsed_nanoseconds ) / 1000000000
	updated_boid.last_updated = now
	
	//	Everything is mildly attracted to the mouse pointer.
	var acceleration Vector
	
	if ( attractor_position.distance_to( from_boid.position ) < attraction_distance ) {
		acceleration = attractor_position.minus( from_boid.position ).divided_by( from_boid.position.distance_to( attractor_position ) ).times( attraction_factor * attractor_polarity )
	}
	
	//	Find nearby boids, and flock with them
	max_relevant_distance := math.Max( Cohesion_Distance, math.Max( Separation_Distance, Alignment_Distance ) )
	from_state.index.boids_near( from_boid, max_relevant_distance, func( boid* Boid, distance float64 ) {
		diff_vector := boid.position.minus( from_boid.position )

		if distance < Separation_Distance {
			acceleration.add( diff_vector.times( -Separation_Factor ) )
		}
		
		if distance < Alignment_Distance {
			acceleration.add( diff_vector.times( Cohesion_Factor ) )
		}
		
		if distance < Alignment_Distance {
			acceleration.add( boid.velocity.times( Alignment_Factor ) )
		}
	} )
	
	//	Apply acceleration limit
	square_acceleration_magnitude := acceleration.square_magnitude()
	if ( square_acceleration_magnitude > ( Acceleration_Limit * Acceleration_Limit ) ) {
		acceleration = acceleration.times( Acceleration_Limit / math.Sqrt( square_acceleration_magnitude ) )
	}
	
	//	Apply acceleration to velocity
	updated_boid.velocity.add( acceleration.times( elapsed_seconds ) )
	
	//	Apply speed limit
	square_speed := updated_boid.velocity.square_magnitude()
	if ( square_speed > ( Speed_Limit * Speed_Limit ) ) {
		updated_boid.velocity = updated_boid.velocity.times( Speed_Limit / math.Sqrt( square_speed ) )
	}
	
	//	Apply velocity
	updated_boid.position.add( updated_boid.velocity.times( elapsed_seconds ) )
	
	//	Wraparound
	for updated_boid.position.x < visible_area.left {
		updated_boid.position.x += visible_area.width()
	}
	
	for updated_boid.position.x > visible_area.right {
		updated_boid.position.x -= visible_area.width()
	}
	
	for updated_boid.position.y < visible_area.top {
		updated_boid.position.y += visible_area.height()
	}
	
	for updated_boid.position.y > visible_area.bottom {
		updated_boid.position.y -= visible_area.height()
	}
	
	out_channel <- updated_boid
}

func create_boid( out_channel chan *Boid ) {
	new_boid := new( Boid )
	new_boid.position.x = rand.Float64() * World_Size - Half_World_Size;
	new_boid.position.y = rand.Float64() * World_Size - Half_World_Size;
	new_boid.last_updated = time.Now()
	new_boid.color_index = rand.Intn( len( boid_colors ) )
	new_boid.color_factor = 0.5 + ( rand.Float32() * 1.0 )

	out_channel <- new_boid;
}

func on_close( window *glfw.Window ) {
	running = false;
}

func on_resize( window *glfw.Window, width, height int ) {
	if height == 0 {
		height = 1
	}
	
	window_width = width
	window_height = height
	
	//	Always ensure at least -Half_World_Size to Half_World_Size is visible on each axis.
	if width > height {
		relative_width := float64( width ) / float64( height );
		visible_area.left = -relative_width * Half_World_Size
		visible_area.right = relative_width * Half_World_Size
		visible_area.top = -Half_World_Size
		visible_area.bottom = Half_World_Size
	} else {
		relative_height := float64( height ) / float64( width );
		visible_area.left = -Half_World_Size
		visible_area.right = Half_World_Size
		visible_area.top = -relative_height * Half_World_Size
		visible_area.bottom = relative_height * Half_World_Size
	}

	gl.MatrixMode( gl.PROJECTION )
	gl.LoadIdentity()
	gl.Ortho( visible_area.left, visible_area.right, visible_area.bottom, visible_area.top, 0, 1 );

	gl.MatrixMode( gl.MODELVIEW )
	gl.LoadIdentity()
}

func on_key( window *glfw.Window, key glfw.Key, scancode int, action glfw.Action, modifiers glfw.ModifierKey ) {
	if action == glfw.Release {
		return
	}

	switch key {
	case glfw.KeyEscape:
		running = false
	
	case glfw.KeyMinus:
		if ( population_size > 100 ) {
			population_size -= 100
		}
	
	case glfw.KeyEqual, glfw.KeyKpAdd:
		if ( population_size < 20000 ) {
			population_size += 100
		}
	
	case glfw.KeyA:
		attractor_polarity = -attractor_polarity;
	
	case glfw.KeyComma:
		if ( attraction_factor > 100 ) {
			attraction_factor /= 10
		}
	
	case glfw.KeyPeriod:
		if ( attraction_factor < 100000 ) {
			attraction_factor *= 10
		}
	}
}

func on_mouse_move( window *glfw.Window, pixel_x, pixel_y float64 ) {
	attractor_position.x = ( float64( pixel_x ) / float64( window_width ) * visible_area.width() ) + visible_area.left
	attractor_position.y = ( float64( pixel_y ) / float64( window_height ) * visible_area.height() ) + visible_area.top
}

func configure_gl() {
	gl.Disable( gl.CULL_FACE )
}

func draw_state( state *State ) {
	gl.Clear( gl.COLOR_BUFFER_BIT )
	gl.LoadIdentity()
	
	//	Draw the boids
	gl.PointSize( 3 )
	for _, boid := range state.boids {
		draw_boid( boid )
	}
	
	//	Draw the attractor
	gl.PointSize( 5 )
	gl.Color4f( 1, 0, 0, 1 )
	gl.Begin( gl.POINTS )
	gl.Vertex2f( float32( attractor_position.x ), float32( attractor_position.y ) )
	gl.End();
	
	now := time.Now()
	elapsed_nanoseconds := now.Sub( last_frame_time )
	elapsed_seconds := float64( elapsed_nanoseconds ) / 1000000000
	average_frame_time = average_frame_time * 0.99 + elapsed_seconds * 0.01
	last_frame_time = now
	
	gl.Color4f( 1, 1, 1, 1 )
	font.Printf( 10, 10, "Frames per second: %d", int( 1 / average_frame_time ) )
	font.Printf( 10, 50, "Boid population: %d (+ / - to adjust)", population_size )

	var attractor_label string
	if ( attractor_polarity < 0 ) {
		attractor_label = "Repel"
	} else {
		attractor_label = "Attract"
	}
	
	font.Printf( 10, 90, "Mouse cursor: %s factor %d (a to toggle, < / > to adjust)", attractor_label, int( attraction_factor ) )
}

func draw_boid( boid *Boid ) {
	angle := boid.velocity.heading() * 57.2957795	//	rads to degrees
	
	gl.PushMatrix()
	gl.Translatef( float32( boid.position.x ), float32( boid.position.y ), 0 )
	gl.Rotatef( float32( angle ), 0, 0, 1 )

	c := &boid_colors[ boid.color_index ]
	gl.Color4f( c.r * boid.color_factor, c.g * boid.color_factor, c.b * boid.color_factor, 1 );
	
	gl.Begin( gl.TRIANGLES )
	gl.Vertex2f( 1, 0 )
	gl.Vertex2f( -1, -0.5 )
	gl.Vertex2f( -1, 0.5 )
	gl.End();	
	
	gl.Begin( gl.POINTS )
	gl.Vertex2f( 0, 0 )
	gl.End()
	
	gl.PopMatrix()
}

