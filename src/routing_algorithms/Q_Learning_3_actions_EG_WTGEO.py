"""
PER CHI LEGGERÃ€: NON SO COME IMPORTARE DA SRC.UTILITIES.CONFIG IL NUMERO 
DI DRONI CHE SONO INIZIALIZZATI LÃ€

L'IDEA ORA È CHE OGNI NODO È UNO STATO E LE AZIONI SONO TUTTI I POSSIBILI INVII AI
NODI VICINI PER UN DETERMINATO PACCHETTO

"""

from operator import ne
import numpy as np
import math
from numpy.core.defchararray import array 
from numpy.core.numeric import NaN
from numpy.lib.type_check import real_if_close
from src.routing_algorithms.georouting import GeoRouting
from src.utilities import utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing
from matplotlib import pyplot as plt


#TO FIX TO OBTAIN THE NUMBER OF DRONES (COSTANT OF CONFIG.PY)
import src.utilities.config as config #try self.simulator.n_drones


#GLOBAL THINGS

#import the library for random values
import random


q = {}



    
#seed for random values, just to have consistence on values 
#TODO
#eliminate after a while
random.seed(2)

#epsilon must be smaller and it represents probability for epsilon-greedy
second_epsilon = 0.05
min_epsilon = 0.05
max_epsilon = 0.25  

georouting_on_next_step = True

#take a random value between 0,1
#epsilon = random.random()
epsilon = 0

#normalize the random value from min_epsilon to max_epsilon
epsilon = min_epsilon + (epsilon * (max_epsilon - min_epsilon))

s = {}

v_star = {}


alpha = 0.5

gamma = 0.9

class AIRouting(BASE_routing):
    
    
    
    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)
        # random generator
        self.rnd_for_routing_ai = np.random.RandomState(self.simulator.seed)
        self.taken_actions = {}  #id event : (old_action)
        setattr(self, 'k', 0)
        

    def feedback(self, drone, id_event, delay, outcome):
        """ return a possible feedback, if the destination drone has received the packet """
        # Packets that we delivered and still need a feedback
        #print(self.drone.identifier, "----------", self.taken_actions)

        # outcome == -1 if the packet/event expired; 0 if the packets has been delivered to the depot
        # Feedback from a delivered or expired packet
        #print(self.drone.identifier, "----------", drone, id_event, delay, outcome)
      
        # Be aware, due to network errors we can give the same event to multiple drones and receive multiple feedback for the same packet!!
        # NOTE: reward or update using the old action!!
        # STORE WHICH ACTION DID YOU TAKE IN THE PAST.
        # do something or train the model (?)
        
        print(self, drone, id_event, delay, outcome)
        
        
            
            
            
            
            
        
        #if the packet isn't still treated, then we train system for it
        if True:
   
            "Doubt: i don't know the utility of this"        
            if id_event in self.taken_actions:
                action = self.taken_actions[id_event]
                del self.taken_actions[id_event]
            "End of doubt"
                
            #if the packet is arrived isn't more valid
            if (outcome == -1):
                
                #we obtain a small reward 
                R = -1
            
                #R = -2
            
            #if the packet arrived
            else:
                          
                #opposite of delay
                temp = 2000 - delay
                
                #we obtain a linear reward based on the delay --> 
                #more the delay and value more close value to 1... 
                #less the delay and value more close to 2 
                temp = (temp - 0) / (2000 - 0)
                #take the reward
                R = 1 + temp 
         
                
                #R = 2 * (1 - delay/self.simulator.event_duration)
                
            #add attempts for the starting drone that has initially the packet
            #TODO
            #maybe also for all the path of packets to incentive themÃ¹            
            
            
            
            
            
            try:
                
                temp_, max_q = s[(drone.identifier, id_event)]
            
            except Exception as e:
                
                temp_, max_q = 0, 0
            
            
            #in this way we also consider the action of return -1 
            if (temp_ == -1):
                
               
                try:
                
                    q[(drone.identifier, 2)] = q[(drone.identifier, 2)] + alpha*(R + gamma* max_q - q[(drone.identifier, 2)] )
                
                except Exception as e:
                    
                    q[(drone.identifier, 2)] = 10
                    
                    q[(drone.identifier, 2)] = q[(drone.identifier, 2)] + alpha*(R + gamma* max_q - q[(drone.identifier, 2)] )
                
                
            #the packet remain to the node
            elif(temp_ == 0):
        
                
                try:
        
                    q[(drone.identifier, 0)] = q[(drone.identifier, 0)] + alpha*(R + gamma* max_q - q[(drone.identifier, 0)])
                
                except Exception as e:
                    
                    q[(drone.identifier, 0)] = 10
                    
                    q[(drone.identifier, 0)] = q[(drone.identifier, 0)] + alpha*(R + gamma* max_q - q[(drone.identifier, 0)])
                
        
            
            #the packet was passed to another drone
            elif (temp_ == 1):
                
                
                try:
                
                    q[(drone.identifier, 1)] = q[(drone.identifier, 1)] + alpha*(R + gamma* max_q - q[(drone.identifier, 1)] )
                
                except Exception as e:
                    
                    q[(drone.identifier, 1)] = 10

                    q[(drone.identifier, 1)] = q[(drone.identifier, 1)] + alpha*(R + gamma* max_q - q[(drone.identifier, 1)] )

                
            
            #q[drone.identifier][self.drone.identifier] = q[drone.identifier][self.drone.identifier] + R
        
            
            
    def relay_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """

        """ arg min score  -> geographical approach, take the drone closest to the depot """
        
        #we take our distance from the depot
        #best_drone_distance_from_depot = util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)
        #best_drone_distance_from_depot = self.compute_distance_to_trajectory_s()
        #initially drone closest is us (we take to the depot the
        #packet without any help)
        best_drone = None
        
        #we initialize an empty list, to find the maximum value
        l = []
        
        #we inizialize a temporanery variable to find maximum value from a list
        m = 0
        
        #we generate a random value, for the epsilon-greedy strategy
        rand = random.random()
        
        #Check the state of the drone, if it should be go back to depet 
        #when the distance from it and its trajectory is minimized
		
        try: 
        	mustGoBack = self.drone.mustGoBack
        except:
        	mustGoBack = False
        	setattr(self.drone, 'mustGoBack', mustGoBack)	
        if mustGoBack:
        	if self.isGoingAway():
        		self.k = self.k + 1
        		print("I'm going back")
        		self.drone.mustGoBack = False
        		return -1
	
        
        #if we are in greedy case
        if (rand < 1 - epsilon):
        
            #we calculate what is the best action to perform, if it is 
            #the action of None, -1 or pass to any neighbour
            
            try:
            
                a = q[(self.drone.identifier, 0)]
            
            except:
                
                q[(self.drone.identifier, 0)] = 10
            
                
                a = q[(self.drone.identifier, 0)]
            
            try:
            
                b = q[(self.drone.identifier, 1)]
            
            except Exception as e:

                q[(self.drone.identifier, 1)] = 10

                b = q[(self.drone.identifier, 1)]

            try:

                c = q[(self.drone.identifier, 2)]
            
            except Exception as e:
                
                q[(self.drone.identifier, 2)] = 10
                
                c = q[(self.drone.identifier, 2)]
            
            
            
            #if the best action is to maintain the packet and remain to
            #our trajectory
            if (a >= b and a >= c):
                
                #we calculate the maximum value of the three possible actions
                #NOT NECESSARY TRY-EXCEPT, EXECUTED JUST BEFORE
                l = [q[(self.drone.identifier, 0)], q[(self.drone.identifier, 1)], q[(self.drone.identifier, 2)]]
                m = max(l)
                
                #we set, in a global variable, that this drone 
                #for this packet has perform the action to maintain
                #the packet and to remain to its trajectory and it is
                #saved also the maximum possible value
                s[(self.drone.identifier, pkd.event_ref.identifier)] = (0, m)
                
                
                
                try:
                    v_star[self.drone.identifier] = v_star[self.drone.identifier] + m
                
                except Exception as e:
                    
                    v_star[self.drone.identifier] = 0
                    v_star[self.drone.identifier] = v_star[self.drone.identifier] + m
                
                
                #do anything
                return None
            
            #if the better action is to go to the depot, then this means
            #that we want do the same of the previous if, in practise
            if (c >= a and c >= b):
                
                #we take the maximum value, for the reward calculation
                #NOT NECESSARY TRY-EXCEPT, EXECUTED JUST BEFORE
                l = [q[(self.drone.identifier, 0)], q[(self.drone.identifier, 1)], q[(self.drone.identifier, 2)]]
                m = max(l)
                
                #we save this result, in practise
                s[(self.drone.identifier, pkd.event_ref.identifier)] = (-1, m)
                
                try:
                    v_star[self.drone.identifier] = v_star[self.drone.identifier] + m
                
                except Exception as e:
                    
                    v_star[self.drone.identifier] = 0
                    v_star[self.drone.identifier] = v_star[self.drone.identifier] + m
                
                
                #at the end we perform the action to go to the depot, so
                #we left the mission for this purpose
                
                if self.isGoingAway():
                     return -1
                try: 
                     self.drone.mustGoBack = True
                except:
                     setattr(self.drone, 'mustGoBack', True)	
                return None
            
            #if the best choice to do is to pass the packet to the neighbors
            if (b >= a and b >= c):
                
                "FIRST PHASE -- TAKE MAX Rs FOR a -- SLIDE 32"
                #we take the maximum value, for the reward calculation
                #NOT NECESSARY TRY-EXCEPT, EXECUTED JUST BEFORE
                l = [q[(self.drone.identifier, 0)], q[(self.drone.identifier, 1)], q[(self.drone.identifier, 2)]]
                m = max(l)
                
                #we initialize two differente variables to do some things
                sum_v_star = 0
                max_v_star = 0
                
                
                "SECOND PHASE PHASE -- SUM OF PROBABILITIES AND V* -- SLIDE 32"
                #we meed to know sum of v* of all neighbors
                for hello_packet, drone_istance in opt_neighbors:
                    
                    sum_v_star = sum_v_star + v_star[drone_istance.identifier]
                
                
                try:
                    
                    #set the v_star attribute
                    v_star[self.drone.identifier] = v_star[self.drone.identifier] + m + gamma*sum_v_star
                
                except Exception as e:
                    
                    v_star[self.drone.identifier] = 0
                    
                    v_star[self.drone.identifier] = v_star[self.drone.identifier] + m + gamma*sum_v_star
                
                
                max_v_star = v_star[self.drone.identifier]
                
                max_action = None

                "THIRD PHASE -- IDENTIFY THE MAX FOR a OF EVERY Q(S',a) (all neighbors)"
                "SLIDE 42"                
                #loop for every neighbors
                for hello_packet, drone_istance in opt_neighbors:
                    

                    #because we must identify max_a Q(S' , a)
                    if (v_star[drone_istance.identifier] > max_v_star):
                        
                        max_v_star = v_star[drone_istance.identifier]     
                        max_action = drone_istance
                        
                #max of possible actions
                return_m = m
                
                "FOURTH PHASE -- SELECT THE BEST NEIGHBOR POSSIBLE, WITH"
                "HIGHEST VALUE LEARNED"
                for i in range(3):
                    
                    for hello_packet, drone_istance in opt_neighbors:
                     
                        try:
                            
                            
                            if (q[(drone_istance.identifier, i)] > return_m):
                             
                             
                                return_m = q[(drone_istance.identifier, i)]
                        
                        
                        except Exception as e:
                            
                            
                            q[(drone_istance.identifier, i)] = 10
                        
                        
                            if (q[(drone_istance.identifier, i)] > return_m):
                             
                                return_m = q[(drone_istance.identifier, i)]
                   
                        
                #save everything for the capturing of the reward in 
                #successive phase of feedback
                if (max_action == None):
                    
                    s[(self.drone.identifier, pkd.event_ref.identifier)] = (0, return_m)
                
                else:
                    
                    s[(max_action.identifier, pkd.event_ref.identifier)] = (1, return_m)
                
                self.drone.mustGoBack = False
                return max_action
                
                
                
                
            
            
        #in the random case (epsilon case)    
        else:
            
            
            
            
            """ arg min score  -> geographical approach, take the drone closest to the depot """
            best_drone_distance_from_depot = util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)
            max_action = None

            
            l = []
            
            
            #we take the maximum value, for the reward calculation
            try:
                
                l.append(q[(self.drone.identifier, 0)])
                
            except Exception as e:
                
                q[(self.drone.identifier, 0)] = 10
                
                l.append(q[(self.drone.identifier, 0)])
                
            
            try:
                
                l.append(q[(self.drone.identifier, 1)])
                
            except Exception as e:
                
                q[(self.drone.identifier, 1)] = 10
                
                l.append(q[(self.drone.identifier, 1)])
                
                
            try:
                
                l.append(q[(self.drone.identifier, 2)])
                
            except Exception as e:
                
                q[(self.drone.identifier, 2)] = 10
                
                l.append(q[(self.drone.identifier, 2)])
                
            
    

            # Action MOVE is identified by -1
            if len(opt_neighbors) == 0:
                
                
                m = max(l)
                
                #we save this result, in practise
                s[(self.drone.identifier, pkd.event_ref.identifier)] = (-1, m)
                
                try:
                    v_star[self.drone.identifier] = v_star[self.drone.identifier] + m
                
                except Exception as e:
                    
                    v_star[self.drone.identifier] = 0
                    v_star[self.drone.identifier] = v_star[self.drone.identifier] + m
                
                
                #at the end we perform the action to go to the depot, so
                #we left the mission for this purpose
                

        
                if self.isGoingAway:
                     return -1
                try: 
                     self.drone.mustGoBack = True
                except:
                     setattr(self.drone, 'mustGoBack', True)	
                return None

            
            "FIRST PHASE -- TAKE MAX Rs FOR a -- SLIDE 32"
            #we take the maximum value, for the reward calculation
            #this is the ELSE part
            
            m = max(l)
            
            #we initialize two differente variables to do some things
            sum_v_star = 0
            max_v_star = 0
            
            
            "SECOND PHASE PHASE -- SUM OF PROBABILITIES AND V* -- SLIDE 32"
            #we meed to know sum of v* of all neighbors
            for hello_packet, drone_istance in opt_neighbors:
                
                
                try:
                    
                
                    sum_v_star = sum_v_star + v_star[drone_istance.identifier]
            
                except Exception as e:
                    
                    v_star[drone_istance.identifier] = 0
                    
                    sum_v_star = sum_v_star + v_star[drone_istance.identifier]
            
            
            
            try:
                
                #set the v_star attribute
                v_star[self.drone.identifier] = v_star[self.drone.identifier] + m + gamma*sum_v_star
            
            except Exception as e:
                
                v_star[self.drone.identifier] = 0
                
                v_star[self.drone.identifier] = v_star[self.drone.identifier] + m + gamma*sum_v_star
            
            
            max_v_star = v_star[self.drone.identifier]
            
            max_action = None

            #max of possible actions
            return_m = m
            
            
            
            


            for hpk, drone_istance in opt_neighbors:
                
                exp_position = hpk.cur_pos  # without estimation, a simple geographic approach
                exp_distance = util.euclidean_distance(exp_position, self.simulator.depot.coords)
                if exp_distance < best_drone_distance_from_depot:
                    best_drone_distance_from_depot = exp_distance
                    max_action = drone_istance
                    
                    try:
                    
                        return_m = q[(drone_istance.identifier, 1)]
                    
                    except Exception as e:
                        
                        q[(drone_istance.identifier, 1)] = 10
                        
                        return_m = q[(drone_istance.identifier, 1)]
                    
                    


            #save everything for the capturing of the reward in 
            #successive phase of feedback
            if (max_action == None):
                
                
                m = max(l)
                
                #we set, in a global variable, that this drone 
                #for this packet has perform the action to maintain
                #the packet and to remain to its trajectory and it is
                #saved also the maximum possible value
                s[(self.drone.identifier, pkd.event_ref.identifier)] = (0, return_m)
                
                
                
                try:
                    v_star[self.drone.identifier] = v_star[self.drone.identifier] + m
                
                except Exception as e:
                    
                    v_star[self.drone.identifier] = 0
                    v_star[self.drone.identifier] = v_star[self.drone.identifier] + m
                
                
                
                
            
            else:
                
                s[(max_action.identifier, pkd.event_ref.identifier)] = (1, return_m)
            
            

            return max_action
    
    
        """
        
        FINE
        
        """
        
       
     
        

    def print(self):
        """
            This method is called at the end of the simulation, can be usefull to print some
                metrics about the learning process
        """
        print(self.k)
        print("sono k")
        pass

    
    
    def compute_extimed_position(self, hello_packet):
        """ estimate the current position of the drone """

        # get known info about the neighbor drone
        hello_message_time = hello_packet.time_step_creation
        known_position = hello_packet.cur_pos
        known_speed = hello_packet.speed
        known_next_target = hello_packet.next_target

        # compute the time elapsed since the message sent and now
        # elapsed_time in seconds = elapsed_time in steps * step_duration_in_seconds
        elapsed_time = (self.simulator.cur_step - hello_message_time) * self.simulator.time_step_duration  # seconds

        # distance traveled by drone
        distance_traveled = elapsed_time * known_speed

        # direction vector
        a, b = np.asarray(known_position), np.asarray(known_next_target)
        if np.linalg.norm(b - a) != 0:
        	v_ = (b - a) / np.linalg.norm(b - a)
        else:
        	v_ = 0

        # compute the expect position
        c = a + (distance_traveled * v_)

        return tuple(c)

    def isGoingAway(self):
    	a = util.euclidean_distance(self.simulator.depot.coords, self.drone.next_target())
    	b = util.euclidean_distance(self.drone.coords, self.drone.next_target())
    	c = util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)
    	import math
    	arg = (b**2 + c**2 - a**2) / (2.0*b*c)
    	if arg > 1.0:
    		arg = 1
    	if arg < -1.0:
    		arg = -1
    	alpha = math.acos(arg)
    	return alpha >= math.pi/2
    	#return self.compute_distance_to_trajectory_s() >= util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)

    def compute_distance_to_trajectory_s(self):
        p1 = np.array([self.drone.coords[0], self.drone.coords[1]])
        p2 = np.array([self.drone.next_target()[0], self.drone.next_target()[1]])
        p3 = np.array([self.drone.depot.coords[0],self.drone.depot.coords[1]])

        
        if np.linalg.norm(p2-p1) != 0:
        	distance = np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)
        	if distance > 0:
        		return distance
        	else:
        		return -distance
        else: 
        	return 0

    def compute_distance_to_trajectory(self, hello_packet):

        exp_position = self.compute_extimed_position(hello_packet)
       # exp_position = hello_packet.cur_pos

        #MAYBE IT SHOULD BE p1 = np.array([exp_position[0][0], exp_position[0][1]])
        p1 = np.array([exp_position[0], exp_position[1]])
        p2 = np.array([hello_packet.next_target[0], hello_packet.next_target[1]])
        p3 = np.array([self.drone.depot.coords[0],self.drone.depot.coords[1]])
        
        if np.linalg.norm(p2-p1) != 0:
        	return np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)
        else: 
        	return 0
