# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  


# This is the variable for Windows.  I use this to define the root of my directory structure.
SET(GLUT_ROOT ${BULLET_PHYSICS_SOURCE_DIR}/Glut)

# You shouldn't have to modify anything below this line*

# * - conditions apply
# Edit: CMake reads a 'CMakeLists.txt' from each subfolder from src to generate a Makefile 
# in the build folder and redirects output to the build folder. Changing the Makefile
# in the build folder doesn't seem to have an effect. We need to link pthreads and IrrKlang.
# Looks like we *have* to modify whatever is beyond this line.
########################################################

INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src 
	../OpenGL 
	${GLUT_INCLUDE_DIR}
)

LINK_LIBRARIES(
OpenGLSupport BulletDynamics  BulletCollision LinearMath    ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} -lpthread -lrt -lIrrKlang -g
)

IF (WIN32)
	ADD_EXECUTABLE(UGTA
		UGTA.cpp
		main.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
	)
	
	
ELSE()
	ADD_EXECUTABLE(UGTA
		UGTA.cpp
		main.cpp
	)
ENDIF()


IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(UGTA PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(UGTA PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(UGTA PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
