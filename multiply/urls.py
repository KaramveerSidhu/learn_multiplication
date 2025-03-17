from django.urls import path
from . import views

urlpatterns = [
    path('', views.mutliply_view, name='multiply'),
]