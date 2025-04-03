from django.shortcuts import render
from django.http import JsonResponse
import random
from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status
import rospy
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState

def mutliply_view(request):
    return render(request, 'multiply.html')

class MyApiView(APIView):
    def get(self, request):
        data = {"message": "Hello from Django API!"}
        return Response(data, status=status.HTTP_200_OK)
    
def generate_questions(request):
    questions = []
    for _ in range(9):
        num1 = random.randint(1, 10)
        num2 = random.randint(1, 10)
        question = f"What is {num1} x {num2}?"
        answer = num1 * num2
        questions.append({"question": question, "answer": answer, "num1": num1, "num2": num2})
    return JsonResponse({"questions": questions})

# Ensure ROS is initialized only once
if not rospy.core.is_initialized():
    rospy.init_node('qtrobot_speaker', anonymous=True)

# Create publishers for QTrobot speech, gesture, ...
speech_publisher = rospy.Publisher('/qt_robot/behavior/talkText', String, queue_size=10)
gesture_publisher = rospy.Publisher('/qt_robot/gesture/play', String, queue_size=10)
gesture_publisher2 = rospy.Publisher('/qt_robot/gesture/list', String, queue_size=10)
print("LIISSSTT", gesture_publisher2)
emotionShow_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)

def board_ready(request):
    welcomeMsg = String()
    welcomeMsg.data = "Your Tic Tac Toe board, is ready!"
    speech_publisher.publish(welcomeMsg)

    return JsonResponse({"speech": "board ready"})

def dictate_question(request):
    num1 = request.GET.get('num1')
    num2 = request.GET.get('num2')

    dictateMsg = String()
    dictateMsg.data = f"What, is {num1} times {num2}"
    speech_publisher.publish(dictateMsg)

    return JsonResponse({"speech": "dictate question"})

head_yaw_pos = 0
head_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=10)
rospy.sleep(3.0)

def state_callback(msg):
    global head_yaw_pos
    head_yaw_pos = msg.position[msg.name.index("HeadYaw")]

rospy.Subscriber('/qt_robot/joints/state', JointState, state_callback)

def correct_ans(request):
    num1 = request.GET.get('num1')
    num2 = request.GET.get('num2')

    num1 = int(num1)
    num2 = int(num2)

    dictateMsg = String()
    dictateMsg.data = f"Yess! {num1 * num2}, is the correct answer."
    speech_publisher.publish(dictateMsg)

    pitch_deg = 15.0
    href = Float64MultiArray()
    href.data = [0, pitch_deg]
    head_pub.publish(href)
    rospy.sleep(0.2)

    pitch_deg = 0.0
    href = Float64MultiArray()
    href.data = [0, pitch_deg]
    head_pub.publish(href)
    rospy.sleep(1)

    return JsonResponse({"speech": "correct ans"})

def incorrect_ans(request):
    num1 = request.GET.get('num1')

    num1 = int(num1)

    emotionShow_pub.publish("QT/sad")

    rospy.sleep(1.5)

    dictateMsg = String()
    dictateMsg.data = f"Nope!"
    speech_publisher.publish(dictateMsg)

    head_yaw_ref = 15.0
    href = Float64MultiArray()
    href.data = [head_yaw_ref, 0]
    head_pub.publish(href)
    rospy.sleep(0.4)

    head_yaw_ref = -15.0
    href = Float64MultiArray()
    href.data = [head_yaw_ref, 0]
    head_pub.publish(href)
    rospy.sleep(0.4)

    head_yaw_ref = 0.0
    href = Float64MultiArray()
    href.data = [head_yaw_ref, 0]
    head_pub.publish(href)
    rospy.sleep(0.4)

    dictateMsg = String()
    dictateMsg.data = f"Sorry, {num1} is not the correct answer."
    speech_publisher.publish(dictateMsg)

    return JsonResponse({"speech": "incorrect ans"})

def winner(request):
    player = request.GET.get('player')

    rospy.sleep(5)

    dictateMsg = String()
    dictateMsg.data = f"Wowww!"
    speech_publisher.publish(dictateMsg)

    rospy.sleep(2)
    emotionShow_pub.publish("QT/happy")
    gesture_publisher.publish("QT/happy")

    rospy.sleep(5)

    dictateMsg = String()
    dictateMsg.data = f"Congratulations, winner is, player, {player}."
    speech_publisher.publish(dictateMsg)

    rospy.sleep(4)

    gesture_publisher.publish("QT/clapping")

    return JsonResponse({"speech": "winner"})

def isdraw(request):
    dictateMsg = String()
    dictateMsg.data = f"Haha! The game is a draw, you both played very well."
    speech_publisher.publish(dictateMsg)

    return JsonResponse({"speech": "isdraw"})

def welcome(request):
    # Welcome message
    welcomeMsg = String()
    welcomeMsg.data = "Welcome!"
    speech_publisher.publish(welcomeMsg)
    gesture_publisher.publish("QT/hi")
    rospy.sleep(5)

    instruction0 = String()
    instruction0.data = "Let's play tic tac maths"
    speech_publisher.publish(instruction0)
    rospy.sleep(3)

    # Instruction 1
    instruction1 = String()
    instruction1.data = "The game, is played on, a Tic Tac Toe board."
    speech_publisher.publish(instruction1)
    rospy.sleep(5)

    # Instruction 2
    instruction2 = String()
    instruction2.data = "To claim a box, you must solve, a multiplication question correctly."
    speech_publisher.publish(instruction2)
    rospy.sleep(7)

    # Instruction 3
    instruction3 = String()
    instruction3.data = "The first player, to get three boxes in a row, horizontally, vertically, or diagonally, wins!"
    speech_publisher.publish(instruction3)
    rospy.sleep(9)

    # Instruction 4
    instruction4 = String()
    instruction4.data = "Have fun, and sharpen your multiplication skills!"
    speech_publisher.publish(instruction4)
    rospy.sleep(5)

    return JsonResponse({"speech": "instructions dictated"})

def visual(request):
    num1 = request.GET.get('num1')
    num2 = request.GET.get('num2')
    num1 = int(num1)
    num2 = int(num2)

    # Step 1: Welcome message
    welcomeMsg = String()
    welcomeMsg.data = f"Let's learn, multiplication, visually! Now, we will learn {num1} times {num2}."
    speech_publisher.publish(welcomeMsg)
    rospy.sleep(9)

    # Step 2: Explain rows
    rowsMsg = String()
    rowsMsg.data = f"First, we will create {num1} rows. Each row, represents one group of {num2}."
    speech_publisher.publish(rowsMsg)
    gesture_publisher.publish("QT/arm_front")
    rospy.sleep(9)

    # Step 3: Explain columns
    columnsMsg = String()
    columnsMsg.data = f"Next, we will create {num2} columns. Each column, represents one item in each group."
    speech_publisher.publish(columnsMsg)
    rospy.sleep(9)

    # Step 4: Explain the grid
    gridMsg = String()
    gridMsg.data = f"Now, we will fill the grid, with {num1 * num2} blocks."
    speech_publisher.publish(gridMsg)
    rospy.sleep(6)

    # Step 5: Explain counting
    countingMsg = String()
    countingMsg.data = f"Finally, count all the blocks in the grid. That will be the answer, to {num1} times {num2}."
    speech_publisher.publish(countingMsg)
    rospy.sleep(8)

    return JsonResponse({"speech": "visual learning steps explained"})

def get_ordinal(n):
        if 10 <= n % 100 <= 20:
            suffix = "th"
        else:
            suffix = {1: "st", 2: "nd", 3: "rd"}.get(n % 10, "th")
        return f"{n}{suffix}"

def addition(request):
    num1 = request.GET.get('num1')
    num2 = request.GET.get('num2')
    num1 = int(num1)
    num2 = int(num2)

    welcomeMsg = String()
    welcomeMsg.data = f"Let's learn, multiplication, using, addition! Now, we will learn, {num1} times {num2}."
    speech_publisher.publish(welcomeMsg)
    rospy.sleep(10)

    conceptMsg = String()
    conceptMsg.data = f"In this case, multiplication, means, adding the number, {num1}, a total of, {num2} times."
    speech_publisher.publish(conceptMsg)
    rospy.sleep(9)

    for i in range(1, num2 + 1):
        ordinal = get_ordinal(i)
        stepMsg = String()
        stepMsg.data = f"Step {i}: Add {num1} for the {ordinal} time.."
        speech_publisher.publish(stepMsg)
        rospy.sleep(6)

    encourageMsg = String()
    encourageMsg.data = f"So, you can add, {num1}, {num2} times, to find the answer. You can do it!"
    speech_publisher.publish(encourageMsg)
    rospy.sleep(5)

    return JsonResponse({"speech": "addition steps explained"})