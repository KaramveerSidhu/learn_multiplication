from django.shortcuts import render

def mutliply_view(request):
    return render(request, 'multiply.html')
