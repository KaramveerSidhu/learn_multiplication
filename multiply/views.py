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