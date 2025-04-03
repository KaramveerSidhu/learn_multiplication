from django.urls import path
from . import views
from .views import MyApiView

urlpatterns = [
    # path('', views.mutliply_view, name='multiply'),
    path('multiply/', MyApiView.as_view(), name='multiply-api'),
    path('generate-questions/', views.generate_questions, name='generate-questions'),
    path('board-ready/', views.board_ready, name='board-ready'),
    path('dictate-question/', views.dictate_question, name='dictate-question'),
    path('correct-ans/', views.correct_ans, name='correct-ans'),
    path('incorrect-ans/', views.incorrect_ans, name='incorrect-ans'),
    path('winner/', views.winner, name='winner'),
    path('isdraw/', views.isdraw, name='isdraw'),
    path('welcome/', views.welcome, name='welcome'),
    path('visual/', views.visual, name='visual-learning'),
    path('addition/', views.addition, name='addition-learning'),
]