import copy
import json
import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"laptop"))
from chat_core import ChatSession, OpenAIInterpreter, validated_intent

class FakeRobot:
    def __init__(self):
        self.current = dict(control_epoch=0, route_index=1, next_junction="D", speed_scale=1.0)
        self.commands = []
    def status(self):
        return copy.deepcopy(self.current)
    def send(self, action, **kwargs):
        self.commands.append((action,kwargs))
        if action=="stop":
            self.current["control_epoch"]+=1
        return dict(accepted=True,id="test",reason="Applied")

class Interpreter:
    def __init__(self, plan):
        self.plan = plan
        self.inputs = []
    def interpret(self, text, status, history):
        self.inputs.append((text,status,history))
        return self.plan

def intent(action="slow_down", **kw):
    return dict(action=action,explanation="Reduce speed slightly.",
                speed_scale=kw.get("speed_scale"),turn=kw.get("turn"))

class ChatTests(unittest.TestCase):
    def test_multiple_action_fields_are_rejected(self):
        bot=FakeRobot()
        chat=ChatSession(bot,Interpreter(intent("speed_up",turn="left")))
        with self.assertRaises(ValueError):
            chat.chat("faster and left")
        self.assertEqual(bot.commands,[])

    def test_status_uses_current_robot_data_instead_of_model_claims(self):
        bot=FakeRobot()
        bot.current.update(state="following",obstacle_stop=True)
        model=Interpreter(dict(intent("status"),explanation="The bot is moving fast"))
        reply=ChatSession(bot,model).chat("What is happening?")
        self.assertIn("Stopped for an obstacle",reply)
        self.assertIn("Next planned junction: D",reply)
        self.assertNotIn("moving fast",reply)
        self.assertEqual(bot.commands,[])

    def test_context_and_previous_ack_are_supplied(self):
        bot=FakeRobot()
        model=Interpreter(intent())
        chat=ChatSession(bot,model)
        chat.chat("Take it a little easier")
        chat.chat("A little more")
        self.assertEqual(len(model.inputs[-1][2]),1)
        self.assertIn("Accepted", model.inputs[-1][2][0]["result"])
        self.assertEqual(model.inputs[-1][1]["next_junction"],"D")
        self.assertEqual(bot.commands[-1][0],"slow_down")

    def test_turn_is_bound_to_original_junction_and_stop_epoch(self):
        bot=FakeRobot()
        chat=ChatSession(bot,Interpreter(intent("turn",turn="right")))
        chat.chat("Take the next right")
        self.assertEqual(bot.commands[-1],("turn",dict(value="right",
            expected_route_index=1,expected_next_junction="D",expected_control_epoch=0)))

    def test_ambiguous_request_sends_no_command(self):
        bot=FakeRobot()
        chat=ChatSession(bot,Interpreter(intent("clarify")))
        chat.chat("Do the thing")
        self.assertEqual(bot.commands,[])

    def test_invalid_model_command_sends_nothing(self):
        bot=FakeRobot()
        chat=ChatSession(bot,Interpreter(intent("speed_scale",speed_scale=50)))
        with self.assertRaises(ValueError):
            chat.chat("faster")
        self.assertEqual(bot.commands,[])

    def test_missing_key_and_refusal_send_nothing(self):
        bot=FakeRobot()
        model=OpenAIInterpreter()
        model.key=""
        with self.assertRaises(RuntimeError):
            ChatSession(bot,model).chat("slow down")
        self.assertEqual(bot.commands,[])

    def test_literal_stop_does_not_need_model_or_status(self):
        bot=FakeRobot()
        model=Interpreter(intent())
        chat=ChatSession(bot,model)
        chat.chat("Stop!")
        self.assertEqual(bot.commands,[("stop",{})])
        self.assertEqual(model.inputs,[])

    def test_stop_cancels_slow_interpretation(self):
        bot=FakeRobot()
        entered,release=threading.Event(),threading.Event()
        class SlowInterpreter:
            def interpret(self,*args):
                entered.set()
                release.wait(2)
                return intent("continue")
        chat=ChatSession(bot,SlowInterpreter())
        worker=threading.Thread(target=lambda:chat.chat("let us proceed"))
        worker.start()
        self.assertTrue(entered.wait(1))
        chat.stop()
        release.set()
        worker.join()
        self.assertEqual(bot.commands,[("stop",{})])

    def test_rejected_command_is_not_reported_as_applied(self):
        bot=FakeRobot()
        bot.send=lambda *a,**kw:dict(accepted=False,reason="No right exit")
        chat=ChatSession(bot,Interpreter(intent("turn",turn="right")))
        self.assertTrue(chat.chat("right next").startswith("Not applied"))

    def test_api_schema_and_incomplete_response(self):
        model=OpenAIInterpreter(key="test-only")
        with patch("chat_core.request_json", return_value={
                "status":"completed","output":[{"type":"message","content":[
                    {"type":"output_text","text":json.dumps(intent())}]}]}) as request:
            result=model.interpret("more",{},[])
            self.assertEqual(result["action"],"slow_down")
            payload=request.call_args[0][1]
            self.assertFalse(payload["store"])
            self.assertTrue(payload["text"]["format"]["strict"])
        with patch("chat_core.request_json",return_value={"status":"incomplete"}):
            with self.assertRaises(RuntimeError):
                model.interpret("more",{},[])

if __name__=="__main__":
    unittest.main()
