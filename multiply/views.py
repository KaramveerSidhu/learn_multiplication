from django.shortcuts import render

def mutliply_view(request):
    return render(request, 'multiply.html')

from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status

class MyApiView(APIView):
    def get(self, request):
        data = {"message": "Hello from Django API!"}
        return Response(data, status=status.HTTP_200_OK)