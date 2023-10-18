import os
import openai

openai.api_key = os.getenv("OPENAI_API_KEY")


prompt = "식당 손님이 주문한 불고기, 김밥, 김치에 대해서 30 단어 이내의 연속된 문장으로 식당 손님에게 간단한 멘트를 해줘"
print("Prompt: " + prompt )

response = openai.ChatCompletion.create(
#    model="gpt-4",
    model="gpt-3.5-turbo",
    messages=[
        {"role": "system", "content": "You are a helpful server of a restaurant."}, 
		{"role": "user", "content": prompt},
#       {"role": "user", "content": "Who won the world series in 2020?"},
#       {"role": "assistant", "content": "The Los Angeles Dodgers won the World Series in 2020."},
#       {"role": "user", "content": "Where was it played?"}
    ],
#	temperature=0,
)

response_content = response['choices'][0]['message']['content']
print("Response: " + response_content )

prompt2 = "오렌지가 추가되었을 때 20 단어 이내의 연속된 문장으로 식당 손님에게 간단한 멘트를 해줘"
print("Prompt: " + prompt2 )

response = openai.ChatCompletion.create(
#    model="gpt-4",
    model="gpt-3.5-turbo",
    messages=[
        {"role": "system", "content": "You are a helpful assistant."}, 
		{"role": "user", "content": prompt},
        {"role": "assistant", "content": response_content},
		{"role": "user", "content": prompt2 }
    ],
#	temperature=0,
)

response_content = response['choices'][0]['message']['content']
print("Response: " + response_content )
