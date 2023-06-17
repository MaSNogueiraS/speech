import rospy
from std_msgs.msg import String

class KeywordDetector(object):
    def __init__(self):
        rospy.init_node('keyword_detector')
        self.publisher_ = rospy.Publisher('keywords_detected', String, queue_size=10)
        self.text_publisher = rospy.Publisher('last_text', String, queue_size=10)  # New publisher for the last text message
        self.subscription = rospy.Subscriber(
            'whisper_recogs',
            String,
            self.listener_callback
        )
        self.last_msg_data = None

    def listener_callback(self, msg):
        # Skip this message if it's the same as the last one we processed
        if msg.data == self.last_msg_data:
            return

        # Update our record of the last message we processed
        self.last_msg_data = msg.data

        # Publish the last text message
        self.text_publisher.publish(String(data=self.last_msg_data))

        init_keywords = ['Hera', 'Herra', 'era']  # Initialization keywords
        question_starts = ['who', 'when', 'where', 'what', 'how', 'why', 'can you', 'could you']
        direction_keywords = ['go', 'move', 'turn', 'stop']
        request_keywords = ['take', 'bring']
        task_keywords = ['take out the garbage', 'carry my luggage']
        # other_keywords = [
        #     ['keyword1', 'keyword2', 'keyword3'],  # List 1
        #     ['keyword4', 'keyword5', 'keyword6'],  # List 2
        #     ['keyword7', 'keyword8', 'keyword9'],  # List 3
        #     ['keyword10', 'keyword11', 'keyword12'],  # List 4
        #     ['keyword13', 'keyword14', 'keyword15'],  # List 5
        # ]

        if any(keyword in msg.data for keyword in init_keywords):
            if any(keyword in msg.data for keyword in direction_keywords) and not msg.data.strip().endswith('?'):
                self.publisher_.publish(String(data='Following a direction'))
            elif any(keyword in msg.data for keyword in request_keywords):
                self.publisher_.publish(String(data='Fulfilling a request'))
            elif any(keyword in msg.data for keyword in task_keywords):
                self.publisher_.publish(String(data='Completing a task'))
            elif any(msg.data.strip().startswith(keyword) for keyword in question_starts) or msg.data.strip().endswith('?'):
                self.publisher_.publish(String(data='Answering a question'))
            # else:
            #     for i, keywords in enumerate(other_keywords, 1):
            #         if any(keyword in msg.data for keyword in keywords):
            #             self.publisher_.publish(String(data=f'Detected from list {i}: {", ".join(keywords)}'))
            #             break

def main():
    keyword_detector = KeywordDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
