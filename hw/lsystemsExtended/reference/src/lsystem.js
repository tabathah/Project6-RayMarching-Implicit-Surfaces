const THREE = require('three')

export function Rule(prob, str)
{
	this.probability = prob; // The probability that this Rule will be used when replacing a character in the grammar string
	this.successorString = str; // The string that will replace the char that maps to this Rule
}

function Node(symbol, iteration) // Node for a linked list of grammar symbols
{
	this.next = null;
	this.prev = null;
	this.character = symbol;
	this.predecessor = null;
	this.iteration = iteration;
	this.pos = new THREE.Vector3(0, 0, 0);
	this.scale = new THREE.Vector3(1, 1, 1);
}

function LinkedList()
{
	this.head = null;
	this.tail = null;
}

function symmetricallyLink(node1, node2)
{
	node1.next = node2;
	node2.prev = node1;
}

// Takes in the node we want to replace
// and randomly chooses and applies a rule to it
// Returns the node from which our expander should continue expanding
//            						Node   Iteration Map of rules
function ApplyRandomRule(linkedList, node, iteration, grammar)
{
	var symbol = node.character;
	var rulesArray = grammar[symbol];

	if(rulesArray == null)
	{
		return;
	}

	var unifRand = Math.random();
	var tmp = 0;
	var rule = null;

	for(var i = 0; i < rulesArray.length; i++)
	{
		tmp += rulesArray[i].probability;
		if(unifRand < tmp)
		{
			rule = rulesArray[i];
			break;
		}
	}
    //console.log(rule.successorString);
	ReplaceNode(linkedList, node, rule.successorString);
}

export function StringToLinkedList(input_string) {
	var result = new LinkedList();

	var root_node = new Node(input_string[0]);

	// "pointer" to current node?
	var current_node = root_node;

	for (var i = 1; i < input_string.length; ++i) {
		var new_node = new Node(input_string[i]);
		symmetricallyLink(current_node, new_node);
		current_node = new_node;
	}
	result.head = root_node;
	result.tail = current_node;
	return result;
}

export function LinkedListToString(linkedList)
{
	var result = "";
	var currentNode;
	for(currentNode = linkedList.head; currentNode != null; currentNode = currentNode.next)
	{
		result = result.concat(currentNode.character);
	}
	return result;
}

//Given the node to be replaced, insert a sub-linked-list that represents replacementString
function ReplaceNode(linkedList, node, replacementString)
{
	var lList = StringToLinkedList(replacementString);
	var finalNext = node.next; // The head of the linked list we're going to append after lList

	if(node.prev == null)
	{
		linkedList.head = lList.head;
	}
	else
	{
		symmetricallyLink(node.prev, lList.head);
	}

	if(finalNext == null)
	{
		linkedList.tail = lList.tail;
	}
	else
	{
		symmetricallyLink(lList.tail, finalNext);
	}
}

export default function Lsystem(axiom, grammar, iterations)
{
	// default LSystem
	this.axiom = "C";
	this.grammar = {};
	this.grammar['C'] = [
		new Rule(1.0, 'SCC')
	];
	this.iterations = 0; 
	
	// Setup axiom
	if (typeof axiom !== "undefined") {
		this.axiom = axiom;
	}

	// Setup grammar
	if (typeof grammar !== "undefined") {
		this.grammar = Object.assign({}, grammar);
	}
	
	// Setup iterations
	if (typeof iterations !== "undefined") {
		this.iterations = iterations;
	}

	// Update parameters
	this.UpdateAxiom = function(axiom) 
	{
		// Setup axiom
		if (typeof axiom !== "undefined")
		{
			this.axiom = axiom;
		}
	}

	// Return results of lsystem after num iterations
	this.DoIterations = function(num)
	{
		var currStringLL = StringToLinkedList(this.axiom);
		for(var i = 0; i < num; i++)
		{
			for(var currNode = currStringLL.head; currNode != null; currNode = currNode.next)
			{
				ApplyRandomRule(currStringLL, currNode, i, this.grammar);
			}
		}

		return currStringLL;
	}
}