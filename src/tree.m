classdef tree < handle

    properties (SetAccess = public)
        children = cell(1, 1)
        parent = zeros(1, 1)
        node_num = 1
        max_nodes = 5000
        new_node = struct
    end

    methods (Access = public)

        function init(this)
            this.parent = zeros(this.max_nodes, 1);
            this.children = cell(this.max_nodes, 1);
            this.node_num = 1;
            this.new_node.id_parent = -1;
        end

        function insert_node(this)
            this.new_node.id = this.node_num;
            this.parent(this.new_node.id, 1) = this.new_node.id_parent;
            this.children{this.new_node.id} = [];

            if this.new_node.id_parent > 0
                this.children{this.new_node.id_parent}(end + 1) = this.new_node.id;
            end

            this.node_num = this.node_num + 1;
        end

        function mapping = delete_node(this, id_delete)
            
        end

        function change_parent(this, id, new)
            old = this.parent(id);
            peer = this.children{old};
            this.children{old} = peer(~ismember(peer, id));
            this.children{new}(end + 1) = id;
            this.parent(id, 1) = new;
        end

        function ids = get_ancestor(this, prev)
            ids = [];
            ids(1, 1) = prev;
            ind = 2;
            prev = this.parent(prev, 1);

            while prev > 0
                ids(ind, 1) = prev;
                ind = ind + 1;
                prev = this.parent(prev, 1);
            end

        end

        function offspring = get_offspring(this, id, offspring)

            if isempty(this.children{id})
                offspring = [];
            else
                tmp = this.children{id};
                offspring = [offspring tmp];

                for i = 1:length(tmp)
                    offspring = this.get_offspring(tmp(i), offspring);
                end

            end

        end

    end

end
