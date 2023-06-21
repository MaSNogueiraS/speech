import rospy
from std_msgs.msg import String
import openai
from collections import deque

#Author: MatS
#Contact: mateus.scarpelli03@gmail.com

class AnswerFromWhisper(object):
    def __init__(self):
        rospy.init_node('answer_from_whisper')
        self.model = "gpt-3.5-turbo"
        self.prompt = ""
        self.conversation = deque([{"role": "system", "content": "Você é a HERA (Assistente Robótica de Ambiente Residencial), um robô de serviço gentil e educado projetado para realizar interação e cooperação entre humanos e robôs, desenvolvido pela equipe RoboFEI@Home do Centro Universitário FEI. O nome Hera foi inspirado na deusa grega protetora. A equipe RoboFEI@Home é atualmente campeã mundial na Robocup Thailand."}], maxlen=11)
        
        self.text_subscriber = rospy.Subscriber(
            'last_text',  
            String,
            self.text_callback
        )
        
        self.keyword_response_subscriber = rospy.Subscriber(
            'keywords_detected',   
            String,  
            self.keyword_response_callback
        )
        
        self.last_text_data = None
        self.last_processed_text = None
    
    def text_callback(self, msg):
        self.last_text_data = msg.data

    def keyword_response_callback(self, msg):
        if msg.data == 'Answering a question' and self.last_text_data:
            # If the last_text_data is the same as the last processed text, do nothing
            if self.last_text_data == self.last_processed_text:
                return
            self.answer(self.last_text_data)
            # Store the last_text_data as the last processed text
            self.last_processed_text = self.last_text_data
    
    def generate_error_message(self, error_context):
        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[{"role": "system", "content": f"A user made a request but there was a problem. {error_context}"},{"role": "user", "content": "What should I say?"}],
                max_tokens=50,
                temperature=0.6
            )
            error_message = response['choices'][0]['message']['content'].strip()
            return error_message
        except openai.api_errors.APIError as e:
            return "Desculpe, encontrei um problema ao processar a sua solicitação."

    def generate_acknowledgement(self, context):
        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[{"role": "system", "content": ""},{"role": "user", "content": context}],
                max_tokens=50,
                temperature=0.6
            )
            acknowledgement = response['choices'][0]['message']['content'].strip()
            return acknowledgement
        except openai.api_errors.APIError as e:
            return "Sorry, I have a small problem with the ai that process the text responses and generate dynamic aknolodgment"

    def answer(self, text):
        self.prompt = text
        openai.api_key = "api_key here"
        self.conversation.append({"role": "user", "content": self.prompt})

        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=list(self.conversation),
                max_tokens=300,
                temperature=0.5
            )

            answer = response['choices'][0]['message']['content'].strip()
            self.conversation.append({"role": "assistant", "content": answer})

            return answer

        except openai.api_errors.APIError as e:
            error_message = self.generate_error_message("I couldn't generate a response to the user's question.")
            print(e)
            self.talk.speak_text(self, error_message)
            return error_message

def main():
    answer_node = AnswerFromWhisper()
    rospy.spin()

if __name__ == '__main__':
    main()
