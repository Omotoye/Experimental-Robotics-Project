#!/usr/bin/env python

import rospy

# the oracle service messages
from exprob_msgs.srv import Knowledge, KnowledgeResponse

# armor custom messages
from armor_msgs.srv import ArmorDirective, ArmorDirectiveList, ArmorDirectiveListRequest
from armor_msgs.msg import _ArmorDirectiveReq

# for generating random hint
import random

from modules.exception_handling import *  


__author__ = "Alessio Capitanelli"
__copyright__ = "Copyright 2016, ArmorPy"
__license__ = "GNU"
__version__ = "1.0.0"
__maintainer__ = "Alessio Capitanelli"
__email__ = "alessio.capitanelli@dibris.unige.it"
__status__ = "Development"



class KnowledgeManager:
    def __init__(self, client_id, reference_name):
        self.reference_name = reference_name
        self.client_id = client_id
        self.timeout = 10
        self._service_name = "/armor_interface_srv"
        self.owl_file_path = "/root/Desktop/cluedo_ontology.owl"
        self.iri = "http://www.emarolab.it/cluedo-ontology"
        self._handle = rospy.ServiceProxy(self._service_name, ArmorDirective)
        self.hypo = rospy.get_param("/hints")
        self.load_ref_from_file(
            self.owl_file_path,
            self.iri,
            buffered_manipulation=True,
            reasoner="PELLET",
            buffered_reasoner=True,
            mounted=False,
        )
        rospy.Service("/knowledge_srv", Knowledge, self.knowledge_clbk)

    def knowledge_clbk(self, msg):
        response = KnowledgeResponse()

        if msg.goal == "update":
            self.get_hint_data(msg.hint_id)
            self.add_ind_to_class(self.ind_name, self.class_name)
            self.call_class_disjoint(self.class_name)
            self.call_reasoner()
            self.add_objectprop_to_ind(
                self.objectprop_name, self.hypo_name, self.ind_name
            )
            self.call_class_disjoint(self.class_name)
            self.call_reasoner()
            response.result = "updated"

        elif msg.goal == "hypo check":
            self.call_reasoner()
            good_hypo = []
            _completed = self.ind_b2_class("COMPLETED")
            _inconsistent = self.ind_b2_class("INCONSISTENT")
            if len(_completed) > 0:
                for hypothesis in _completed:
                    if hypothesis not in _inconsistent:
                        good_hypo.append(hypothesis)
            if len(good_hypo) > 0:
                response.result = "hypo found"
                response.hypo_ids = good_hypo
            else:
                response.result = "no hypo found"

        elif msg.goal == "announce hypo":
            who, what, where = self.get_hypo_data(msg.hypo_id)
            print("Hey there Oracle!, I have a correct hypothesis for you.\n")
            print(f"{who} performed the killing at the {where} with a {what}")
            response.result = "announced"
        return response

    def get_hint_data(self, hint_id):
        i, j = list(hint_id)
        (self.hypo_name,) = list(self.hypo[int(i)])
        (self.objectprop_name,) = list(self.hypo[int(i)][self.hypo_name][int(j)])
        self.ind_name = self.hypo[int(i)][self.hypo_name][int(j)][self.objectprop_name]
        if self.objectprop_name == "who":
            self.class_name = "PERSON"
        elif self.objectprop_name == "where":
            self.class_name = "PLACE"
        elif self.objectprop_name == "what":
            self.class_name = "WEAPON"

    def get_hypo_data(self, hypo_id):
        (hypo_name,) = list(self.hypo[int(hypo_id)])

        for items in self.hypo[int(hypo_id)][hypo_name]:
            (key,) = list(items)
            value = items[key]
            if key == "who":
                who = value
            elif key == "what":
                what = value
            elif key == "where":
                where = value
        return who, what, where

    def load_ref_from_file(
        self,
        owl_file_path,
        iri,
        buffered_manipulation=True,
        reasoner="PELLET",
        buffered_reasoner=True,
        mounted=False,
    ):
        """
        Loads an ontology into armor from an .owl file.

        Args:
                owl_file_path (str):
                iri (str):
                buffered_manipulation (bool): set buffered manipulations, default False
                reasoner (str): set which reasoner to use (PELLET, HERMIT, SNOROCKET, FACT), default PELLET
                buffered_reasoner (bool): set if reasoner should be buffered, default False
                mounted (bool): set if the client should be mounted on the reference, default False

        Raises:
                armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
                armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
        """
        try:
            if mounted:
                res = self.call(
                    "LOAD",
                    "FILE",
                    "MOUNTED",
                    [
                        owl_file_path,
                        iri,
                        str(buffered_manipulation),
                        reasoner,
                        str(buffered_reasoner),
                    ],
                )
            else:
                res = self.call(
                    "LOAD",
                    "FILE",
                    "",
                    [
                        owl_file_path,
                        iri,
                        str(buffered_manipulation),
                        reasoner,
                        str(buffered_reasoner),
                    ],
                )

        except rospy.ServiceException:
            raise ArmorServiceCallError(
                "Service call failed upon reference {0} from {1}".format(
                    self._client.reference_name, self._client.client_id
                )
            )

        except rospy.ROSException:
            raise ArmorServiceCallError(
                "Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running."
            )

        if not res.success:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def call_class_disjoint(self, class_name):
        try:
            res = self.call("DISJOINT", "IND", "CLASS", [class_name])

        except rospy.ServiceException as e:
            raise ArmorServiceCallError(
                "Service call failed upon calling disjoint to class {1}: {2}".format(
                    class_name, e
                )
            )

        except rospy.ROSException:
            raise ArmorServiceCallError(
                "Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running."
            )

        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def call_reasoner(self):
        try:
            res = self.call("REASON", "", "", [])

        except rospy.ServiceException as e:
            raise ArmorServiceCallError(
                "Service call failed upon calling reasoner: {2}".format(e)
            )

        except rospy.ROSException:
            raise ArmorServiceCallError(
                "Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running."
            )

        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def ind_b2_class(self, class_name):
        """
        Query the list of all individuals belonging to a class.

        Args:
                class_name (str): a class in the ontology

        Returns:
                list(str): the list of individual belonging to the class

        Raises:
                armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails.
                armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error.
        """
        try:
            res = self.call("QUERY", "IND", "CLASS", [class_name])

        except rospy.ServiceException:
            raise ArmorServiceCallError(
                "Service call failed upon querying individuals belonging to class {0}".format(
                    class_name
                )
            )

        except rospy.ROSException:
            raise ArmorServiceCallError(
                "Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running."
            )

        if res.success:
            return res.queried_objects
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def add_ind_to_class(self, ind_name, class_name):
        """
        Add an individual to a class.

        Args:
                ind_name (str): individual to be added to the class.
                class_name (str): individual will be added to this class. It will be created a new class if it does not exist.

        Returns:
                bool: True if ontology is consistent, else False

        Raises:
                armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
                armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error

        Note:
                It returns the boolean consistency state of the ontology. This value is not updated to the last operation
                if you are working in buffered reasoner or manipulation mode!
        """
        try:
            res = self.call("ADD", "IND", "CLASS", [ind_name, class_name])

        except rospy.ServiceException as e:
            raise ArmorServiceCallError(
                "Service call failed upon adding individual {0} to class {1}: {2}".format(
                    ind_name, class_name, e
                )
            )

        except rospy.ROSException:
            raise ArmorServiceCallError(
                "Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running."
            )

        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def add_objectprop_to_ind(self, objectprop_name, ind_name, value_obj_name):
        """
        Add an object property to an individual. If the object property to be assigned does not exist, it will be created.

        Args:
                objectprop_name (str): name of the object property to assign.
                ind_name (str): individual to assign the data property value.
                value_obj_name (str): name of the individual to be used as property value.

        Returns:
                bool: True if ontology is consistent, else False

        Raises:
                armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
                armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error

        Note:
                It returns the boolean consistency state of the ontology. This value is not updated to the last operation
                if you are working in buffered reasoner or manipulation mode!
        """
        try:
            res = self.call(
                "ADD", "OBJECTPROP", "IND", [objectprop_name, ind_name, value_obj_name]
            )

        except rospy.ServiceException:
            err_msg = "Service call failed upon adding property {0} to individual {1}.".format(
                objectprop_name, ind_name
            )
            raise ArmorServiceCallError(err_msg)

        except rospy.ROSException:
            raise ArmorServiceCallError(
                "Cannot reach ARMOR self: Timeout Expired. Check if ARMOR is running."
            )

        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def call(self, command, first_spec, second_spec, args_list):
        req = self._prepare_request(command, first_spec, second_spec, args_list)
        rospy.wait_for_service(self._service_name, self.timeout)
        res = self._handle(req).armor_response
        return res

    def _prepare_request(self, command, first_spec, second_spec, args_list):
        req = _ArmorDirectiveReq.ArmorDirectiveReq()
        req.client_name = self.client_id
        req.reference_name = self.reference_name
        req.command = command
        req.primary_command_spec = first_spec
        req.secondary_command_spec = second_spec
        req.args = args_list
        return req


if __name__ == "__main__":
    rospy.init_node("knowledge_manager")
    KnowledgeManager(rospy.get_name(), "cluedo_game")
    rospy.spin()
