from datetime import datetime
import json
import re
from typing import Any, Text, Dict, List, Optional
from rasa_sdk import Tracker, FormValidationAction
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet
import os
import random
from emotion_recognition.emotion_recognition import EmotionRecognition

# Class for emotion recognition
emotion_recognition = EmotionRecognition()

# File path where the informations are stored
FILE_PATH_PEOPLE_INFO = "contest_result.txt"
FILE_PATH_CONTEST = "contest_group.txt"

def load_data_json(file_path):
    """
    Load data from the specified file.
    Returns a dictionary containing people data or an empty dictionary in case of an error.
    """
    try:
        with open(file_path, "r") as file:
            data = json.load(file)
        return data
    except FileNotFoundError:
        return {"people": []}

def filter_people(gender=None, bag=None, hat=None, line=None):
    """
    Filters people based on the specified criteria (gender, presence of a bag, hat, or crossed line).

    Args:
        gender (str): Gender of the person (e.g., "male", "female").
        bag (bool): Indicates if the person has a bag.
        hat (bool): Indicates if the person is wearing a hat.
        line (int): Specifies a line crossed by the person.

    Returns:
        list: A list of people who match the specified criteria.
    """
    data = load_data_json(FILE_PATH_PEOPLE_INFO).get("people", [])

    if line is not None:
        line = int(line)

    return [
        person for person in data
        if (gender is None or person.get("gender") == gender)
        and (bag is None or person.get("bag") == bag)
        and (hat is None or person.get("hat") == hat)
        and (line is None or line in person.get("trajectory", []))

    ]

def read_contest_file(dispatcher1):
    """
    Reads group data from the contest.

    Args:
        dispatcher1 (CollectingDispatcher): Rasa dispatcher.

    Returns:
        list: A list of tuples containing group information.
    """
    if not os.path.exists(FILE_PATH_CONTEST):
        dispatcher1.utter_message(text="The file containing the groups could not be found.")
        print("ERROR: File not found.")
        return []

    gruppi = []
    with open(FILE_PATH_CONTEST, 'r') as file:
        for linea in file:
            parti = linea.strip().split(',')
            gruppo = parti[0]
            score = parti[1]
            membri = ", ".join(parti[2:])
            gruppi.append((gruppo, score, membri))
    return gruppi

# Emotional Analysis
class ActionAnalyzeFace(Action):
    def name(self) -> Text:
        return "action_analyze_face"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        # Send message containing the emotion
        emotion = emotion_recognition.process_emotion()
        dispatcher.utter_message(text=emotion)
        return []

# DATE AND TIME
class ActionShowTime(Action):
    def name(self) -> Text:
        return "action_show_time"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        """
        Displays the current time to the user.

        Args:
            dispatcher (CollectingDispatcher): Dispatcher to send responses.
            tracker (Tracker): Rasa tracker.
            domain (dict): Domain of the actions.

        Returns:
            list: Empty list of events.
        """
        current_time = datetime.now().strftime("%H:%M")
        dispatcher.utter_message(text=f"The current time is {current_time}")
        return []
        
class ActionAskDate(Action):
    def name(self) -> str:
        return "action_ask_date"

    def run(self, dispatcher, tracker, domain) -> List[Dict[str, Any]]:
        """
        Displays the current date to the user.

        Args:
            dispatcher (CollectingDispatcher): Dispatcher to send responses.
            tracker (Tracker): Rasa tracker.
            domain (dict): Domain of the actions.

        Returns:
            list: Empty list of events.
        """
        today_date = datetime.now().strftime("%A, %d %B %Y")  # Format: Monday, 01 January 2025
        dispatcher.utter_message(text=f"Today is {today_date}. It's a great day to get things done.")
        return []

class ActionCompareGender(Action):
    def name(self) -> Text:
        return "action_compare_gender"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        """
        Compares the count of people based on their gender and provides a response.

        Args:
            dispatcher (CollectingDispatcher): Dispatcher to send responses.
            tracker (Tracker): Rasa tracker.
            domain (dict): Domain of the actions.

        Returns:
            list: Empty list of events.
        """
        # Extract 'gender' entities from the tracker
        entities = tracker.latest_message.get("entities", [])
        genders = [entity['value'] for entity in entities if entity['entity'] == 'gender']

        # Check if at least two genders are provided
        if len(genders) < 2:
            dispatcher.utter_message(text="I need information about two genders to make a comparison. Could you tell me which genders you want to compare?")
            return []

        # Retrieve the mentioned genders
        gender1, gender2 = genders[0], genders[1]

        # Filter people by gender
        people_gender1 = filter_people(gender=gender1)
        people_gender2 = filter_people(gender=gender2)

        # Compare the counts of people by gender
        if len(people_gender1) > len(people_gender2):
            message = (
                f"Based on the current data, there are more {gender1}s, with a total of {len(people_gender1)}, "
                f"compared to {len(people_gender2)} {gender2}s."
            )
        elif len(people_gender1) < len(people_gender2):
            message = (
                f"It seems that {gender2}s are more numerous, with a count of {len(people_gender2)}, "
                f"compared to {len(people_gender1)} {gender1}s."
            )
        else:
            message = (
                f"The number of {gender1}s and {gender2}s is equal, with both having {len(people_gender1)} individuals."
            )

        # Send the response message to the dispatcher
        dispatcher.utter_message(text=message)
        return []

class ActionCompareAttributes(Action):
    def name(self) -> Text:
        return "action_compare_attributes"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        """
        Compares the count of people based on attributes such as carrying bags or wearing hats.

        Args:
            dispatcher (CollectingDispatcher): Dispatcher to send responses.
            tracker (Tracker): Rasa tracker.
            domain (dict): Domain of the actions.

        Returns:
            list: Empty list of events.
        """
        # Extract entities from the user's message
        entities = tracker.latest_message.get("entities", [])
        has_bag = next((entity['value'] for entity in entities if entity['entity'] == 'has_bag'), None)
        has_hat = next((entity['value'] for entity in entities if entity['entity'] == 'has_hat'), None)
        gender = next((entity['value'] for entity in entities if entity['entity'] == 'gender'), None)

        # Check if at least two attributes are present
        if not (has_bag and has_hat):
            dispatcher.utter_message(text="I need at least one attribute to compare, like 'bags' or 'hats'. Could you provide more details?")
            return []

        # Filter people based on the attributes and gender (if present)
        people_with_bags = filter_people(bag=True, gender=gender) if has_bag else []
        people_with_hats = filter_people(hat=True, gender=gender) if has_hat else []

        # Count people for each attribute
        count_bags = len(people_with_bags)
        count_hats = len(people_with_hats)

        # Determine the subject (gender or "people")
        subject = f"{gender}s" if gender else "people"

        # Determine which attribute is more common
        if count_bags > count_hats:
            message = f"There are more {subject} carrying bags, with a total of {count_bags}, compared to {count_hats} wearing hats."
        elif count_bags < count_hats:
            message = f"There are more {subject} wearing hats, with a total of {count_hats}, compared to {count_bags} carrying bags."
        else:
            message = f"The number of {subject} carrying bags and wearing hats is the same, with {count_bags} for each."

        # Send the response message
        dispatcher.utter_message(text=message)
        return []

# PERSON INFO
class ActionGetIndividualInfo(Action):
    def name(self) -> Text:
        return "action_get_individual_info"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        """
        Retrieves and displays information about a specific individual based on their ID or other attributes.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of events, if any.
        """
        # Extract the individual ID from the latest message's entities
        entities = tracker.latest_message.get("entities", [])
        individual_id = next((entity['value'] for entity in entities if entity['entity'] == 'individual_id'), None)
    
        if not individual_id:
            # Fallback to other attributes if ID is not provided
            gender = tracker.get_slot("gender")
            has_hat = tracker.get_slot("has_hat")
            has_bag = tracker.get_slot("has_bag")
            line_id = tracker.get_slot("line_id")

            if gender is None and has_hat is None and has_bag is None and line_id is None:
                dispatcher.utter_message(text="I couldn't identify the individual. Could you please provide their ID?")
            else:
                # Convert attributes to boolean if present
                bag = True if has_bag is not None else None
                hat = True if has_hat is not None else None
                filtered_people = filter_people(gender=gender, bag=bag, hat=hat, line=line_id)
                
                # Build a response based on filtered results
                if len(filtered_people) == 1:
                   message = "Here are the details:\n"
                else:
                    message = "Here are the details for every person:\n"
                
                for person in filtered_people:
                    message += f"ID: {person.get('id')}, Gender: {person.get('gender')}, Hat: {person.get('hat')}, Bag: {person.get('bag')}, Trajectory: {person.get('trajectory')}. \n"

                dispatcher.utter_message(text=message)
        else:

            try:
                # Convert the ID into an integer
                individual_id = int(individual_id.replace("id", ""))
            except ValueError:
                dispatcher.utter_message(text="The ID you provided is not valid. Please use a format like 'id1' or 'id2'.")
                return []

            # Load the data
            data = load_data_json(FILE_PATH_PEOPLE_INFO)

            # Search for the individual by ID
            people = data.get("people", [])
            individual = next((person for person in people if person.get("id") == individual_id), None)

            if individual:
                # Prepare and send individual details
                details = ", ".join([f"{key}: {value}" for key, value in individual.items() if key != "id"])
                dispatcher.utter_message(text=f"Here are the details for ID{individual_id}: {details}. Let me know if you need further information.")
            else:
                dispatcher.utter_message(text=f"I couldn't find any information for ID{individual_id}. Please check the ID and try again.")

        return []

#COUNTING PEOPLE
class ActionCountPeople(Action):
    def name(self) -> Text:
        return "action_count_people"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        """
        Counts people based on provided attributes like gender, bag, hat, or line crossed.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of events, including updated slots.
        """
        # Extract all entities from the user's message
        entities = tracker.latest_message.get("entities", [])

        # Initialize variables for attributes
        gender = next((entity['value'] for entity in entities if entity['entity'] == 'gender'), None)
        has_bag = next((entity['value'] for entity in entities if entity['entity'] == 'has_bag'), None)
        has_hat = next((entity['value'] for entity in entities if entity['entity'] == 'has_hat'), None)
        line_id = next((entity['value'] for entity in entities if entity['entity'] == 'line_id'), None)

        # Convert attributes to boolean if present
        bag = True if has_bag is not None else None
        hat = True if has_hat is not None else None
        line = True if line_id is not None else None

        # Filter people based on provided criteria
        filtered_people = filter_people(gender=gender, bag=bag, hat=hat, line=line_id)
        
        # Build the dynamic message
        if len(filtered_people) == 0:
            gender_text = f"{gender}s" if gender else "people"
            bag_text = "carrying bags" if bag else ""
            hat_text = "wearing hats" if hat else ""
            conjunction = " and " if bag and hat else ""
            line_text = f" who has crossed the line {line_id}" if line else ""
            message = (
                f"I couldn't find any {gender_text} {bag_text}{conjunction}{hat_text}{line_text} "
                f"in the shopping mall. Could you provide more details or try again?"
            )
        else:
            count = len(filtered_people)
            gender_text = f"{gender}s" if gender else "people"
            bag_text = "carrying bags" if bag else ""
            hat_text = "wearing hats" if hat else ""
            conjunction = " and " if bag and hat else ""
            line_text = f" who has crossed the line. {line_id}" if line else ""
            
            # List of alternative closing phrases
            closing_phrases = [
                "Let me know if you need anything else!",
                "Feel free to ask if you have more questions!",
                "I'm here if you need more help!",
                "Just ask if there's anything else you'd like to know!",
                "Don't hesitate to reach out if you have more queries!"
            ]

            # Select a random closing phrase
            closing_text = random.choice(closing_phrases)

            # Construct the message
            message = (
                f"I found {count} {gender_text} {bag_text}{conjunction}{hat_text}{line_text} "
                f"in the shopping mall. {closing_text}"
            )

        # Send the message
        dispatcher.utter_message(text=message)
        return [SlotSet("gender", gender), SlotSet("has_bag", has_bag), SlotSet("has_hat", has_hat), SlotSet("line_id", line_id)]

# PERSON INFO
class ActionGetIndividualInfoAttributes(Action):
    def name(self) -> Text:
        return "action_get_individual_info_attributes"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        """
        Retrieves and displays information about a specific individual based on their ID or other attributes.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of events, if any.
        """
        # Extract the individual ID from the latest message's entities
        entities = tracker.latest_message.get("entities", [])
        individual_id = next((entity['value'] for entity in entities if entity['entity'] == 'individual_id'), None)
        gender = next((entity['value'] for entity in entities if entity['entity'] == 'gender'), None)
        has_bag = next((entity['value'] for entity in entities if entity['entity'] == 'has_bag'), None)
        has_hat = next((entity['value'] for entity in entities if entity['entity'] == 'has_hat'), None)
        line_id = next((entity['value'] for entity in entities if entity['entity'] == 'line_id'), None)
        
        try:
            # Convert the ID into an integer
            individual_id = int(individual_id.replace("id", ""))
        except ValueError:
            dispatcher.utter_message(text="The ID you provided is not valid. Please use a format like 'id1' or 'id2'.")
        
        # Convert attributes to boolean if present
        bag = True if has_bag is not None else None
        hat = True if has_hat is not None else None
        line = True if line_id is not None else None

        # Filter people based on provided criteria
        filtered_people = filter_people()
        person = next((person for person in filtered_people if person['id'] == individual_id), None)


        if person is None:
            dispatcher.utter_message(text=f"No individual with ID {individual_id} found.")
            return []

        # Build response based on specified attributes
        response = "\n"

        # Check gender
        if gender:
            if person['gender'] == gender:
                response += f"\n Yes, the individual with ID {individual_id} is {gender}."
            else:
                response += f"\n No, the individual with ID {individual_id} is not a {gender}, is a {person['gender']}."

        # Check if carrying a bag
        if has_bag is not None:
            if person['bag']:
                response += f"\n Yes, the individual with ID {individual_id} is carrying a bag."
            else:
                response += f"\n No, the individual with ID {individual_id} is not carrying a bag."

        # Check if wearing a hat
        if has_hat is not None:
            if person['hat']:
                response += f"\n Yes, the individual with ID {individual_id} is wearing a hat."
            else:
                response += f"\n No, the individual with ID {individual_id} is not wearing a hat."

        # Check if crossed a specific line
        if line_id is not None:
            try:
                line_id = int(line_id)
                if line_id in person['trajectory']:
                    response += f"\n Yes, the individual with ID {individual_id} has crossed line {line_id}."
                else:
                    response += f"\n No, the individual with ID {individual_id} has not crossed line {line_id}."
            except ValueError:
                response += "\n- The line ID provided is not valid."

        # Send the constructed response to the user
        dispatcher.utter_message(text=response)
        return []

# Action to provide additional information about counted people
class ActionCountPeopleMoreInfo(Action):
    def name(self) -> Text:
        return "action_count_people_more_info"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        """
        Provides a detailed count of people based on various attributes like gender, bags, hats, and crossed lines.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of events, including updated slots.
        """
        # Extract entities or fallback to slot values
        entities = tracker.latest_message.get("entities", [])
        gender = next((entity['value'] for entity in entities if entity['entity'] == 'gender'), tracker.get_slot("gender"))
        has_hat = next((entity['value'] for entity in entities if entity['entity'] == 'has_hat'), tracker.get_slot("has_hat"))
        has_bag = next((entity['value'] for entity in entities if entity['entity'] == 'has_bag'), tracker.get_slot("has_bag"))
        line_id = next((entity['value'] for entity in entities if entity['entity'] == 'line_id'), tracker.get_slot("line_id"))

        # Convert attributes to boolean if applicable
        bag = True if has_bag is not None else None
        hat = True if has_hat is not None else None
        line = True if line_id is not None else None

        # Filter people based on criteria
        filtered_people = filter_people(gender=gender, bag=bag, hat=hat, line=line_id)
        
        # Build the dynamic message
        if len(filtered_people) == 0:
            gender_text = f"{gender}s" if gender else "people"
            bag_text = "carrying bags" if bag else ""
            hat_text = "wearing hats" if hat else ""
            conjunction = " and " if bag and hat else ""
            line_text = f" who has crossed the line {line_id}" if line else ""
            message = (
                f"I couldn't find any {gender_text} {bag_text}{conjunction}{hat_text}{line_text} "
                f"in the shopping mall. Could you provide more details or try again?"
            )
        else:
            count = len(filtered_people)
            gender_text = f"{gender}s" if gender else "people"
            bag_text = "carrying bags" if bag else ""
            hat_text = "wearing hats" if hat else ""
            conjunction = " and " if bag and hat else ""
            line_text = f" who has crossed the line. {line_id}" if line else ""
            
            # Alternative closing phrases
            closing_phrases = [
                "Let me know if you need anything else!",
                "Feel free to ask if you have more questions!",
                "I'm here if you need more help!",
                "Just ask if there's anything else you'd like to know!",
                "Don't hesitate to reach out if you have more queries!"
            ]

            # Randomly select a closing phrase
            closing_text = random.choice(closing_phrases)

            # Construct the response
            message = (
                f"I found {count} {gender_text} {bag_text}{conjunction}{hat_text}{line_text} "
                f"in the shopping mall. {closing_text}"
            )

        # Send the message
        dispatcher.utter_message(text=message)
        return [SlotSet("gender", gender), SlotSet("has_bag", has_bag), SlotSet("has_hat", has_hat), SlotSet("line_id", line_id)] 

# SEARCH PERSON
class ActionSearchPerson(Action):
    def name(self) -> str:
        return "action_search_person"

    def run(self, dispatcher, tracker, domain) -> List[Dict[str, Any]]:
        """
        Searches for a person based on provided attributes such as gender, bag, hat, and trajectory line.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of SlotSet events.
        """
        # Extract entities from the user's message
        entities = tracker.latest_message.get("entities", [])

        # Initialize variables for the attributes
        gender = next((entity['value'] for entity in entities if entity['entity'] == 'gender'), None)
        has_bag = next((entity['value'] for entity in entities if entity['entity'] == 'has_bag'), None)
        has_hat = next((entity['value'] for entity in entities if entity['entity'] == 'has_hat'), None)
        line_id = next((entity['value'] for entity in entities if entity['entity'] == 'line_id'), None)

        # Convert attributes to booleans
        bag = True if has_bag is not None else None
        hat = True if has_hat is not None else None
        line = True if line_id is not None else None

        # Check if sufficient details are provided
        if gender is None and has_bag is None and has_hat is None and line_id is None:
            dispatcher.utter_message(
                text="I need more details to find the person. Could you provide their gender, if they have a bag or a hat, or if the have crossed a line?"
            )
            return [
                SlotSet("person_ids", []),
                SlotSet("gender", gender),
                SlotSet("has_bag", has_bag),
                SlotSet("has_hat", has_hat),
                SlotSet("line_id", line_id),
            ]        
            
        # Filter people based on the given attributes
        filtered_people = filter_people(gender=gender, bag=bag, hat=hat,line=line_id)

        # Collect IDs of filtered people
        person_ids = [person["id"] for person in filtered_people]

        # Generate the response
        if len(filtered_people) == 1:
            person = filtered_people[0]
            last_seen_number = person["trajectory"][-1] if person["trajectory"] else None
            last_seen_name = f"line {last_seen_number}" if last_seen_number else "an unknown location"
            response = (
                f"I found one person: a {person['gender']} with ID {person['id']}. "
                f"{'wearing a hat,' if person['hat'] else ''} "
                f"{'carrying a bag' if person['bag'] else ''}"
                f"{f'who has crossed line {line_id}.' if line_id else ''} "
                f".\n The last position of this person is: {last_seen_name}."
            )
        elif len(filtered_people) > 1:
            last_seen_positions = []
            for person in filtered_people:
                last_seen_number = person["trajectory"][-1] if person["trajectory"] else None
                last_seen_name = f"line {last_seen_number}" if last_seen_number else "an unknown location"
                last_seen_positions.append(
                    f".\nPerson with ID {person['id']}, last seen at {last_seen_name}"
                )
            response = (
                f"I found multiple people matching these characteristics: "
                f"{'a ' + gender if gender else 'people'}, "
                f"{'wearing a hat,' if hat else ''} "
                f"{'carrying a bag' if bag else ''}"
                f"{f'who has crossed line {line_id}.' if line_id else ''} "
                + ".\n".join(last_seen_positions)
                + ".\nIf you have additional details, I can help narrow down the search."
            )
        else:

            if hat is None and bag is None:
                response = (f" Absolutely no one has been through here.")
            else:
                response = (
                    f"I couldn't find anyone matching these characteristics: "
                    f"{'a ' + gender if gender else 'people'}, "
                    f"{'wearing a hat,' if hat else ''} "
                    f"{'carrying a bag' if bag else ''}. "
                    f"{f'who has crossed line {line_id}.' if line_id else ''} "
                    f".\nCould you provide more details or try again?"
                )
        # Send the message to the dispatcher
        dispatcher.utter_message(text=response)

        # Save the IDs of found people in a slot and the other attributes
        return [SlotSet("person_ids", person_ids), SlotSet("gender", gender), SlotSet("has_bag", has_bag), SlotSet("has_hat", has_hat), SlotSet("line_id", line_id)]

class ActionMoreInfo(Action):
    def name(self) -> Text:
        return "action_more_info"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        """
        Provides additional information about people matching specific attributes like gender, hats, bags, and trajectory line.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of SlotSet events.
        """
        # Extract entities from the user's message or fallback to slot values
        entities = tracker.latest_message.get("entities", [])
        gender = next((entity['value'] for entity in entities if entity['entity'] == 'gender'), tracker.get_slot("gender"))
        has_hat = next((entity['value'] for entity in entities if entity['entity'] == 'has_hat'), tracker.get_slot("has_hat"))
        has_bag = next((entity['value'] for entity in entities if entity['entity'] == 'has_bag'), tracker.get_slot("has_bag"))
        line_id = next((entity['value'] for entity in entities if entity['entity'] == 'line_id'), tracker.get_slot("line_id"))

        # Convert attributes to booleans
        bag = True if has_bag is not None else None
        hat = True if has_hat is not None else None
        line = True if line_id is not None else None
       
        # Check if sufficient details are provided
        if gender is None and has_bag is None and has_hat is None and line_id is None:
            dispatcher.utter_message(
                text="I need more information to find the person. Could you tell me their gender, if they have a bag, a hat, or if they have crossed a line?"
            )
            return [
                SlotSet("person_ids", []),
                SlotSet("gender", gender),
                SlotSet("has_bag", has_bag),
                SlotSet("has_hat", has_hat),
                SlotSet("line_id", line_id),
            ]

        filtered_people = filter_people(gender=gender, bag=bag, hat=hat,line=line_id)
        
        # Collect IDs of filtered people
        person_ids = [person["id"] for person in filtered_people]

        # Generate the response
        if len(filtered_people) == 1:
            person = filtered_people[0]
            last_seen_number = person["trajectory"][-1] if person["trajectory"] else None
            last_seen_name = f"line {last_seen_number}" if last_seen_number else "an unknown location"
            response = (
                f"I found one person: a {person['gender']} with ID {person['id']}. "
                f"{'wearing a hat,' if person['hat'] else ''} "
                f"{'carrying a bag' if person['bag'] else ''}"
                f"{f'who has crossed line {line_id}.' if line_id else ''} "
                f".\nThe last position of this person is: {last_seen_name}."
            )
        elif len(filtered_people) > 1:
            last_seen_positions = []
            for person in filtered_people:
                last_seen_number = person["trajectory"][-1] if person["trajectory"] else None
                last_seen_name = f"line {last_seen_number}" if last_seen_number else "an unknown location"
                last_seen_positions.append(
                    f".\nPerson with ID {person['id']}, last seen at {last_seen_name}"
                )
            response = (
                f"I found multiple people matching these characteristics: "
                f"{'a ' + gender if gender else 'people'}, "
                f"{'wearing a hat,' if hat else ''} "
                f"{'carrying a bag' if bag else ''}"
                f"{f'who has crossed line {line_id}.' if line_id else ''} "
                + ".\n".join(last_seen_positions)
                + ".\nIf you have additional details, I can help narrow down the search."
            )
        else:
            response = (
                f"I couldn't find anyone matching these characteristics: "
                f"{'a ' + gender if gender else 'people'}, "
                f"{'wearing a hat,' if hat else ''} "
                f"{'carrying a bag' if bag else ''}. "
                f"{f'who has crossed line {line_id}.' if line_id else ''} "
                f".\nCould you provide more details or try again?"
            )

        # Send the message to the dispatcher
        dispatcher.utter_message(text=response)

        # Save the IDs of found people in a slot
        return [SlotSet("person_ids", person_ids), SlotSet("gender", gender), SlotSet("has_bag", has_bag), SlotSet("has_hat", has_hat), SlotSet("line_id", line_id)]

class ActionShowTrajectory(Action):
    def name(self) -> str:
        return "action_show_trajectory"

    def run(self, dispatcher, tracker, domain) -> List[Dict[str, Any]]:
        """
        Displays the trajectory of people based on saved IDs.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of events, if any.
        """
        # Retrieve person IDs saved in the slot
        person_ids = tracker.get_slot("person_ids")

        if not person_ids:
            dispatcher.utter_message(
                text="I don't have any information about the person you're referring to. Could you provide more informations about the person?"
            )
            return []

        # Filter people data based on saved IDs
        people_data = filter_people()
        filtered_people = [person for person in people_data if person["id"] in person_ids]

        if not filtered_people:
            dispatcher.utter_message(text="I couldn't find any trajectory information for the given person. I'm sorry.")
            return []

        # Build the response with trajectories
        trajectories = []
        for person in filtered_people:
            trajectory_names = [f"line {loc}" for loc in person["trajectory"]]
            if trajectory_names:
                trajectories.append(
                    f"Person with ID {person['id']} moved along the following path: {' , '.join(trajectory_names)}."
                )
            else:
                trajectories.append(f"Person with ID {person['id']} has no recorded trajectory.")

        response = "Here are the trajectories I found:\n" + "\n".join(trajectories)

        # Send the message to the dispatcher
        dispatcher.utter_message(text=response)
        return []

# CONTEST
class ActionGetGroupMembers(Action):
    def name(self) -> Text:
        return "action_get_group_members"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        """
        Retrieves and displays the members of a specific group based on the group number.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of events, if any.
        """
        # Read group data from the file
        av_group = read_contest_file(CollectingDispatcher)

        if not av_group:
            dispatcher.utter_message(text="No data was loaded from the file.")
            print("ERROR: No data in the file.")
            return []

        # Retrieve the group number from the slot
        group = tracker.get_slot("group_number")

        if not group:
            response = (
                "I'm not sure which group you're asking about. "
                "Could you please provide the group number so I can assist you better?"
            )
            dispatcher.utter_message(text=response)
            return []

        # Search for the group in the loaded data
        group_members = None
        for gruppo, _, membri in av_group:
            if gruppo == group:
                group_members = membri
                break

        # Generate the response based on the search result
        if group_members:
            response = (
                f"The members of group {group} are : {''.join(group_members)}. "
                "Let me know if you need information about another group!"
            )
        else:
            response = (
                f"I couldn't find any information about group {group}. "
                "Are you sure the group number is correct? You can try providing it again."
            )

        # Send the response to the dispatcher
        dispatcher.utter_message(text=response)
        return []

class ActionGetGroupScore(Action):
    def name(self) -> Text:
        return "action_ask_group_score"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        """
        Retrieves and displays the score of a specific group based on the group number.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of events, if any.
        """
        # Read group data from the file
        av_group = read_contest_file(CollectingDispatcher)

        if not av_group:
            dispatcher.utter_message(text="No data was loaded from the file.")
            print("ERROR: No data in the file.")
            return []

        # Retrieve the group number from the slot
        group = tracker.get_slot("group_number")

        if not group:
            response = (
                "I'm not sure which group you're asking about. "
                "Could you please provide the group number so I can assist you better?"
            )
            dispatcher.utter_message(text=response)
            return []

        # Search for the group in the loaded data
        group_score = None
        for gruppo, score, _ in av_group:
            if gruppo == group:
                group_score = score
                break

        # Generate a response based on the search result
        if group_score:
            response = (
                f"The F1-score of group {group} is {group_score}. "
                f"Let me know if you need details about another group!"
            )
        else:
            response = (
                f"I couldn't find any information about group {group}. "
                f"Are you sure the group number is correct? If you'd like, you can try providing it again."
            )

        # Send the response to the dispatcher
        dispatcher.utter_message(text=response)
        return []
 
class ActionGetPositionContest(Action):
    def name(self) -> Text:
        return "action_get_position"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        """
        Retrieves and displays the group information based on its position in the contest rankings.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of events, if any.
        """
        # Read group data from the file
        av_group = read_contest_file(CollectingDispatcher)

        if not av_group:
            dispatcher.utter_message(text="No data was loaded from the file.")
            print("ERROR: No data in the file.")
            return []

        # Retrieve the position entity from the tracker
        posizione = tracker.get_slot("position")

        if not posizione:
            dispatcher.utter_message(
                text="I couldn't understand the requested position. Could you specify it again, for example, 'first' or 'winner'?"
            )
            return []

        # Mapping of positions to numeric rankings
        posizione_mapping = {
	    "winner": 1, "first": 1, "second": 2, "third": 3, "fourth": 4,
	    "fifth": 5, "sixth": 6, "seventh": 7, "eighth": 8, "ninth": 9,
	    "tenth": 10, "eleventh": 11, "twelfth": 12, "thirteenth": 13,
	    "fourteenth": 14, "last": 14,
	    "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9,
	    "ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13, "fourteen": 14,
	    "1": 1, "2": 2, "3": 3, "4": 4, "5": 5, "6": 6, "7": 7, "8": 8, "9": 9, "10": 10,
	    "11": 11, "12": 12, "13": 13, "14": 14
	}

        # Determine the numeric position
        if isinstance(posizione, int):
            position_number = posizione
        else:
            position_number = posizione_mapping.get(posizione.lower())

        # Determine the numeric position
        position_number = posizione_mapping.get(posizione.lower())

        if not position_number or position_number > len(av_group):
            dispatcher.utter_message(
                 text=f"I couldn't determine the requested position '{posizione}' or it is out of range. Please try again with a valid position."
            )
            return [SlotSet("group_number", None)]

        # Retrieve the corresponding group
        try:
            group_number, group_score, group_members = av_group[position_number - 1]  # Accedi alla riga corretta
        except IndexError:
            dispatcher.utter_message(
                text=f"I'm sorry, but I couldn't find a group for the requested position '{posizione}'."
            )
            return [SlotSet("group_number", None)]

        # Generate the response
        response = (
            f"The group ranked {posizione} is Group {group_number}, "
            f"with an F1-score of {group_score}. "
        )

        dispatcher.utter_message(text=response)
        return [SlotSet("group_number", group_number)]

class ActionGetParticipantsCount(Action):
    def name(self) -> Text:
        return "action_get_participants_count"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        """
        Retrieves and displays the total number of participants in the contest.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of events, if any.
        """
        # Read group data from the file
        av_group = read_contest_file(CollectingDispatcher)

        if not av_group:
            dispatcher.utter_message(text="No data was loaded from the file.")
            print("ERROR: No data in the file.")
            return []

        try:
            # Calculate the total number of participants
            totale_partecipanti = sum(len(membri.split(', ')) for _, _, membri in av_group)
            response = f"A total of {totale_partecipanti} participants took part in the contest. Let me know if you need further details about the groups!"
        except Exception as e:
            response = (
                "An error occurred while calculating the number of participants. "
                "Please try again later."
            )
            print(f"ERROR: {str(e)}")

        # Send the response
        dispatcher.utter_message(text=response)
        return []

class ActionGetGroupsCount(Action):
    def name(self) -> Text:
        return "action_get_groups_count"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        """
        Retrieves and displays the total number of groups in the contest.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of events, if any.
        """
        # Read group data from the file
        av_group = read_contest_file(CollectingDispatcher)

        if not av_group:
            dispatcher.utter_message(text="No data was loaded from the file.")
            print("ERROR: No data in the file.")
            return []

        try:
            # Calculate the total number of groups
            total_groups = len(av_group)
            response = f"A total of {total_groups} groups participated in the contest. Let me know if you'd like details about a specific group or its members!"
        except Exception as e:
            response = (
                "An error occurred while calculating the number of groups. "
                "Please try again later."
            )
            print(f"ERROR: {str(e)}")

        # Send the response
        dispatcher.utter_message(text=response)
        return []

class ActionHighestScoreGroup(Action):
    def name(self) -> str:
        return "action_highest_score_group"

    def run(self, dispatcher, tracker, domain) -> List[Dict[str, Any]]:
        """
        Retrieves and displays the group with the highest score.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of events, if any.
        """
        # Read group data from the file
        av_group = read_contest_file(CollectingDispatcher)

        if not av_group:
            dispatcher.utter_message(text="No data was loaded from the file.")
            print("ERROR: No data in the file.")
            return []

        group_number, group_score, _ = av_group[0]

        # Generate the response
        response = (
                f"The group with the highest score is Group {group_number}, "
                f"with an impressive score of {group_score}. "
                f" Let me know if you'd like details about a specific group or its members!"
            )

        # Send the response
        dispatcher.utter_message(text=response)

        return [SlotSet("group_number", group_number)]

class ActionLowestScoreGroup(Action):
    def name(self) -> str:
        return "action_lowest_score_group"

    def run(self, dispatcher, tracker, domain) -> List[Dict[str, Any]]:
        """
        Retrieves and displays the group with the lowest score.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of events, if any.
        """
        # Read group data from the file
        av_group = read_contest_file(CollectingDispatcher)

        if not av_group:
            dispatcher.utter_message(text="No data was loaded from the file.")
            print("ERROR: No data in the file.")
            return []

        group_number, group_score, _ = av_group[len(av_group) - 1]

        # Generate the response
        response = (
            f"The group with the lowest score is Group {group_number} "
            f" Let me know if you'd like details about a specific group or its members!"
            )

        # Send the response
        dispatcher.utter_message(text=response)

        return [SlotSet("group_number", group_number)]
    
class ActionQueryHighScoringGroups(Action):
    def name(self) -> str:
        return "action_query_high_scoring_groups"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        """
        Retrieves and displays the groups with scores higher or lower than a given value.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict[Text, Any]]: A list of events, if any.
        """
        entities = tracker.latest_message.get("entities", [])
        try:
            # Extract the `f_score` entity and convert it to a float
            f_score = next((ent["value"] for ent in entities if ent["entity"] == "f_score"), None)
            f_score = float(f_score) if f_score is not None else None
        except (ValueError, TypeError):
            f_score = None

        # Ensure f_score is valid (accepting 0 and 1 inclusively)
        if f_score is None or not (0 <= f_score <= 1):
            dispatcher.utter_message(text="Please provide a valid score threshold between 0 and 1, inclusive.")
            return []

        # Check if the user asked for groups with scores below the threshold
        user_message = tracker.latest_message.get("text", "").lower()
        is_lower = "lower" in user_message

        # Read group data from the file
        gruppi = read_contest_file(dispatcher)

        # Filter groups based on the request
        if is_lower:
            gruppi_filtrati = [gruppo for gruppo in gruppi if float(gruppo[1]) < f_score]
            message_prefix = f"A total of {len(gruppi_filtrati)} groups scored below {f_score}."
        else:
            gruppi_filtrati = [gruppo for gruppo in gruppi if float(gruppo[1]) > f_score]
            message_prefix = f"A total of {len(gruppi_filtrati)} groups scored above {f_score}."

        if not gruppi_filtrati:
            dispatcher.utter_message(text=f"No groups scored {'below' if is_lower else 'above'} {f_score}.")
        else:
            message = message_prefix + "\n"
            for gruppo in gruppi_filtrati:
                message += f"Group {gruppo[0]} with a score of {gruppo[1]}.\n"
            dispatcher.utter_message(text=message)

        return []

# CONFIG
class ActionResetSlots(Action):
    def name(self) -> Text:
        return "action_reset_slots"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict]:
        """
        Resets all the slots by setting them to None.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict]: A list of SlotSet events, each resetting a slot to None.
        """        
        return [SlotSet(slot, None) for slot in tracker.slots.keys()]
    
# action out of context
class ActionOutOfContext(Action):
    def name(self):
        return "action_out_of_context"

    def run(self, dispatcher, tracker, domain):
        """
        Responds to out-of-context queries with a default message.

        Args:
            dispatcher (CollectingDispatcher): Used to send responses to the user.
            tracker (Tracker): Tracks the conversation state and context.
            domain (Dict[Text, Any]): The domain of the assistant.

        Returns:
            List[Dict]: An empty list as no slots or events are updated.
        """
        dispatcher.utter_message(text="Sorry, I can’t help with that. Let me know if there’s something else!")
        return []
