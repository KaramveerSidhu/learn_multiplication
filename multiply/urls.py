from django.urls import path
from . import views
from .views import MyApiView

urlpatterns = [
    # path('', views.mutliply_view, name='multiply'),
    path('multiply/', MyApiView.as_view(), name='multiply-api'),
]