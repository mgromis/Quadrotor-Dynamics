 function [X, Y] = getCoords(id, corner,splits, tags)
        [i, j] = find(id == tags);
        if(corner ==1)
            X = .152*(2*i-2)+.152;
            Y = .152*(j-1)+(sum(splits(1:(j))));
        else if (corner ==2)
                X = .152*(2*i-2)+.152;
                Y = .152 + .152*(j-1)+(sum(splits(1:(j))));
            else if (corner == 3)
                X = .152*(2*i-2);
                Y = .152 + .152*(j-1)+sum(splits(1:(j)));
                else if (corner ==4)
                    X = .152*(2*i-2);
                    Y = .152*(j-1)+sum(splits(1:(j)));
                   
                    else
                        X = .152*(2*i-2)+.152/2;
                    Y = .152*(j-1)+sum(splits(1:(j)))+.152/2;
                        
                    end
                    
                end
            end
            
        end
        
        
        
 end



