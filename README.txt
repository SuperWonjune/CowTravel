SimpleScene.py


------------------------------------------------------
1. Implements the UI for control point specification
------------------------------------------------------


controlPoints = []
controlPointIndex = -1

pointsAtCatmul = []

전역 변수로 controlPoints List를 선언해서사용자가 찍는 6개의 소의
좌표를 저장시켜놓고추후 Spline 경로 계산에 활용한다. 그와 동시에
Control Point의 x,y,z 좌표도 저장해놓고 점으로 표시,
이동 경로를 하늘색 선으로 그려서 이동 경로를 알기 쉽게 보이게 한다.

------------------------------------------------------

def onMouseDrag(window, x, y):
	...

	if button == glfw.MOUSE_BUTTON_LEFT:
	    if state == GLFW_DOWN:
	        isDrag = V_DRAG
	        print( "Left mouse down-click at %d %d\n" % (x,y))
	        # start vertical dragging
	    elif state == GLFW_UP and isDrag!=0:
	        isDrag=H_DRAG
	        print( "Left mouse up\n")

	        if not isCowSelected:
	            isCowSelected = True
	            return

	        # add control points
	        controlPoints.append(copy.deepcopy(cow2wld))
	        print("Control Points Added. Total :", len(controlPoints))


	        if len(controlPoints) >= NUM_CONTROL_POINTS:
	            # Duplicate by certain iterations (default 3)
	            controlPoints = controlPoints * NUM_STEP_ITERATIONS
	            # Start the Animation
	            cowMovingInit()

	...

사용자가 마우스를 누르지 않은채로 움직이면 Horizontal하게 움직이고,
꾹 누른 상태로 드래그하면 Vertical하게 이동한다. 그리고 마우스를
놓는 순간 (GLFW_UP) 그 자리에 Cow의 Position이 controlPoints
배열에 저장된다. 이 배열이 정해진 숫자(6개)가 되면 Animation이
시작된다.

------------------------------------------------------

def display():
	...

	# Draw ControlPointing Cows
    if not isCowMoving:
        for pos in controlPoints:
            drawCow(pos, False)
    ...

controlPoints에 저장된 cow들은 Animation이 시작되기 전까지
그 position에 위치하도록 그린다.	


------------------------------------------------------
2. Implements vertical dragging (L-drag)
as well as horizontal dragging (mouse-move)
------------------------------------------------------


if  isDrag==V_DRAG:
	# vertical dragging
	    if cursorOnCowBoundingBox:
	        ray = screenCoordToRay(window, x, y)
	        pp = pickInfo
	        p = Plane(np.array((1, 0, 0)), getTranslation(cow2wld))
	        c = ray.intersectsPlane(p)

	        currentPos = ray.getPoint(c[1])
	        currentPos[2] = getTranslation(cow2wld)[2]
	        # print(pp.cowPickPosition, currentPos)
	        # print(pp.cowPickConfiguration, cow2wld)

	        T = np.eye(4)
	        setTranslation(T, currentPos - getTranslation(cow2wld))
	        cow2wld = T @ cow2wld


소의 Vertical Dragging은 구현하는 방법은 먼저 x축을 normal vector로 가지는
Plane을 Ray와 intersect 시킨다. 그러면 y, z 성분만이 시점상에서 이동하게
되는데 여기서 Z index 부분만 currentPos로 갖고와 translation을 적용한다.


------------------------------------------------------
3. Use the cyclic Catmull-Rom spline curve
which is an interpolating spline
------------------------------------------------------


def cowMovingInit():
    global checkTime, cow2wld, controlPoints, controlPointIndex, isCowMoving, cow2wldDefault
    print("BEGIN ANIMATING...")
    cow2wldDefault = np.copy(controlPoints[0])
    controlPointIndex = -1
    checkTime = glfw.get_time()
    isCowMoving = True

def cowMovingEnd():
    global controlPoints, cow2wld, pointsAtCatmul, isCowMoving, cow2wldDefault
    cow2wld = np.copy(cow2wldDefault)
    controlPoints.clear()
    pointsAtCatmul.clear()
    isCowMoving = False


6개의 Control Point가 그려지면 cowMovingInit()이 실행된다.
Animation에 쓰이는 각종 변수가 초기화되고
glfw.get_time()을 이용해 Timer가 돌아가게 되면서 Animation이 진행된다.

Animation이 다 끝나면 cowMovingEnd()가 호출되고 다시 Control Point를
선택하는 화면으로 돌아가게 된다.

------------------------------------------------------

def cowRide():
	...

	elapsedTime = glfw.get_time() - checkTime

    # next x, y, z position
    next_pos = getNowSplinePoint(elapsedTime)

    ...

    # Move to next Catmul-Rom
    setTranslation(cow2wld, next_pos)

    ...

이동, 회전을 포함한 모든 Animation은 cowRide() 함수 내에서 이뤄진다.
먼저 get_time()을 이용해 한 control Point에서 다음 control Point 사이를 지나는
정해진 시간(여기선 1초)와 현재 경과된 시간의 비율을 이용, spline 식을 계산해
다음에 소가 위치할 장소인 next_pos를 구한다. 그리고 그 next_pos를 이용해
소를 translate 시킨다.


------------------------------------------------------

def getNowSplinePoint(time_elapsed):
    global controlPointIndex, controlPoints, INTERVAL_BETWEEN_CONTROL_POINTS
    t = time_elapsed / INTERVAL_BETWEEN_CONTROL_POINTS

    b0 = (-t + 2 * t * t - t * t * t) / 2.0
    b1 = (2 + -5 * t * t + 3 * t * t * t) / 2.0
    b2 = (t + 4 * t * t - 3 * t * t * t) / 2.0
    b3 = (-t * t + t * t * t) / 2.0

    x = b0 * getControlPoint(controlPointIndex + 0)[0] + \
        b1 * getControlPoint(controlPointIndex + 1)[0] + \
        b2 * getControlPoint(controlPointIndex + 2)[0] + \
        b3 * getControlPoint(controlPointIndex + 3)[0]

    y = b0 * getControlPoint(controlPointIndex + 0)[1] + \
        b1 * getControlPoint(controlPointIndex + 1)[1] + \
        b2 * getControlPoint(controlPointIndex + 2)[1] + \
        b3 * getControlPoint(controlPointIndex + 3)[1]

    z = b0 * getControlPoint(controlPointIndex + 0)[2] + \
        b1 * getControlPoint(controlPointIndex + 1)[2] + \
        b2 * getControlPoint(controlPointIndex + 2)[2] + \
        b3 * getControlPoint(controlPointIndex + 3)[2]

    return vector3(x, y, z)


다음 이동할 점은 controPoints List와 Timer를 참조,
Matrix Multiplcation이 완료된 Spline식을 이용해서 계산한다.


------------------------------------------------------
4. Cow should face forward (yaw orientation)
&&
5. Cow should face upward when going up (pitch orientation) 
------------------------------------------------------

def cowRide():
	...

	elapsedTime = glfw.get_time() - checkTime

    # next x, y, z position
    next_pos = getNowSplinePoint(elapsedTime)

    # Rotate cow toward next_pos
    rotateCowToGivenDirection(getTranslation(cow2wld), next_pos)

    ...


def rotateCowToGivenDirection(current, faceTo):
    global cow2wld

    d = normalize(faceTo - current)

    roll = 0
    pitch = math.asin(d[1])
    yaw = math.atan2(d[2], d[0])

    if yaw < 0:
        pitch = -pitch

    Rx = np.array([[1., 0., 0.],
                   [0., np.cos(pitch), -np.sin(pitch)],
                   [0., np.sin(pitch), np.cos(pitch)]])

    Ry = np.array([[np.cos(yaw), 0., np.sin(yaw)],
                   [0., 1., 0.],
                   [-np.sin(yaw), 0., np.cos(yaw)]])

    Rz = np.array([[np.cos(roll), -np.sin(roll), 0.],
                   [np.sin(roll), np.cos(roll), 0.],
                   [0., 0., 1.]])
    setTransformation(cow2wld, (Ry @ Rx @ Rz).T)



회전도 마찬가지로 cowRide() 함수 내에서 이뤄진다. 앞서 계산했던
next_pos와 Cow의 현재 위치를 참조하여 rotateCowToGivenDirection()을
호출하여 다음 소가 바라볼 direction vector를 계산하고,
direction vector를 활용하여 소의 Pitch와 Yaw를 계산한다.
마지막으로 pitch, yaw를 활용해 각각 축을 기준으로 Transformation을 적용해 회전시킨다.
