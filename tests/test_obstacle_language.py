"""Offline obstacle language never reaches a command receiver."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"laptop"))
from offline_interpreter import OfflineInterpreter
from command_preview import PreviewSession


class ObstacleLanguageTests(unittest.TestCase):
    def test_reports(self):
        for phrase in ("obstacle", "duck", "yellow object blocking the path",
                       "something in the way", "there is a duck ahead",
                       "there's an obstacle in our lane", "a rubber duck",
                       "the duck is blocking the road", "ducks", "there are ducks ahead",
                       "the road is blocked", "something is blocking the way"):
            result=OfflineInterpreter().interpret(phrase)
            self.assertEqual(result.category,"obstacle",phrase)
            self.assertEqual(result.parameters["intent"],"report")
            self.assertFalse(result.parameters["camera_verified"])
            self.assertTrue(result.interpretation_only)

    def test_avoidance_context(self):
        bot=OfflineInterpreter()
        bot.interpret("there is a duck ahead")
        bot.interpret("thanks")
        result=bot.interpret("go around it")
        self.assertEqual(result.parameters["intent"],"avoid")
        self.assertTrue(result.parameters["preserve_route"])
        for phrase in ("avoid the duck","go around the obstacle","bypass the yellow object",
                       "drive around the rubber duck","overtake the duck"):
            result=bot.interpret(phrase)
            self.assertEqual(result.category,"obstacle")
            self.assertEqual(result.parameters["return_lane"],"right")

    def test_negation_hypothetical_compound_and_unrelated(self):
        for phrase in ("don't avoid the duck","there is no obstacle",
                       "if there is a duck go around it","avoid the duck and turn left",
                       "i like yellow ducks","duck under the table"):
            session=PreviewSession()
            session.chat("take the next right")
            result=session.chat(phrase)
            self.assertNotEqual(result.status,"preview_available",phrase)
            self.assertIsNone(session.draft,phrase)

    def test_no_live_mapping_and_clear_context(self):
        session=PreviewSession()
        for phrase in ("duck","go around it","is there a duck ahead"):
            session.chat("speed up")
            result=session.chat(phrase)
            self.assertNotEqual(result.status,"preview_available")
            self.assertIsNone(session.draft)
            with self.assertRaises(ValueError):
                session.record()
        session.chat("duck")
        session.clear()
        self.assertEqual(session.chat("go around it").status,"clarification_needed")

if __name__=="__main__":
    unittest.main()
