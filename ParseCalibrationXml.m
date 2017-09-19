%%  Still under construction.


function [ A_p_b, A_q_b ] = ParseCalibrationXml( xml_file, id_A, id_B)
%ParseCalibrationXml Summary of this function goes here

   tree = xmlread(xml_file);
   theStruct = parseChildNodes(tree);
   
   for i = 1 : length(theStruct.Children)
        
        if strcmp(theStruct.Children(i).Name, 'extrinsic_calibration' ) == 0 
            continue;
        end   
        [does_match, left_to_right] = HasMatchingIDs(theStruct.Children(i).Attributes, id_A, id_B);
        if ~does_match
            continue;
        end
        if left_to_right 
            data_A = theStruct.Children(i).Children(1).Value;
            data_B = theStruct.Children(i).Children(2).Value;
        else 
            data_A = theStruct.Children(i).Children(2).Value;
            data_B = theStruct.Children(i).Children(1).Value;
        end
   end
   theStruct
  
%    if 
%        
%    else if 
%            
%    end
%        
%    theStruct.Children(8).Children(2).Children.Data
end

function [bool, left_to_right] = HasMatchingIDs(Attributes, id1, id2) 

    attr1 = Attributes(1).Value;
    attr2 = Attributes(2).Value;
    
    if id1 == str2double(attr1) && id2 == str2double(attr2) 
        left_to_right = 1;
        bool = 1;
    elseif id2 == str2double(attr1) && id1 == str2double(attr2) 
        left_to_right = 0;
        bool = 1;
    else 
        idx1 = 0;
        idx2 = 0;
        bool = 0;
    end
        
end



% ----- Local function PARSECHILDNODES -----
function children = parseChildNodes(theNode)
% Recurse over node children.
children = [];
if theNode.hasChildNodes
   childNodes = theNode.getChildNodes;
   numChildNodes = childNodes.getLength;
   allocCell = cell(1, numChildNodes);

   children = struct(             ...
      'Name', allocCell, 'Attributes', allocCell,    ...
      'Data', allocCell, 'Children', allocCell);

    for count = 1:numChildNodes
        theChild = childNodes.item(count-1);
        children(count) = makeStructFromNode(theChild);
    end
end

end

% ----- Local function MAKESTRUCTFROMNODE -----
function nodeStruct = makeStructFromNode(theNode)
% Create structure of node info.

nodeStruct = struct(                        ...
   'Name', char(theNode.getNodeName),       ...
   'Attributes', parseAttributes(theNode),  ...
   'Data', '',                              ...
   'Children', parseChildNodes(theNode));

if any(strcmp(methods(theNode), 'getData'))
   nodeStruct.Data = char(theNode.getData); 
else
   nodeStruct.Data = '';
end

end


% ----- Local function PARSEATTRIBUTES -----
function attributes = parseAttributes(theNode)
% Create attributes structure.

attributes = [];
if theNode.hasAttributes
   theAttributes = theNode.getAttributes;
   numAttributes = theAttributes.getLength;
   allocCell = cell(1, numAttributes);
   attributes = struct('Name', allocCell, 'Value', ...
                       allocCell);

   for count = 1:numAttributes
      attrib = theAttributes.item(count-1);
      attributes(count).Name = char(attrib.getName);
      attributes(count).Value = char(attrib.getValue);
   end
end
end